"""Brown/fisheye chest calibration, full-frame remap, and an isolated ROS image topic."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def _first_positive_root(coefficients):
    roots = np.roots(coefficients)
    return min((float(root.real) for root in roots
                if abs(root.imag) < 1e-8 and root.real > 0), default=float('inf'))


def _usable_radius(model, d):
    if model == 'equidistant':
        k1, k2, k3, k4 = d
        # theta_d(theta) has to remain increasing on the central branch.
        theta = _first_positive_root([9*k4, 0, 7*k3, 0, 5*k2, 0, 3*k1, 0, 1])
        return np.tan(theta) if theta < np.pi/2 else float('inf')
    k1, k2, p1, p2, k3 = d
    # The two radial Jacobian eigenvalues must dominate the tangential
    # derivative, whose spectral norm is bounded by 6 * hypot(p1,p2) * r.
    tangential = -6*np.hypot(p1, p2)
    return min(_first_positive_root([k3, 0, k2, 0, k1, tangential, 1]),
               _first_positive_root([7*k3, 0, 5*k2, 0, 3*k1, tangential, 1]))


class ChestRectification:
    def __init__(self, info, focal_scale=1.0):
        self.width, self.height = info['width'], info['height']
        if any(type(n) is not int or not 1 < n <= 8192 for n in (self.width, self.height)):
            raise ValueError('Invalid camera image dimensions')
        if not np.isfinite(focal_scale) or not 0.25 <= focal_scale <= 2:
            raise ValueError('Virtual focal scale must be between 0.25 and 2')
        k = np.asarray(info['K'], dtype=np.float64).reshape(3, 3)
        d = np.asarray(info['D'], dtype=np.float64).reshape(-1)
        model = info['distortion_model']
        if (model not in ('plumb_bob', 'equidistant') or len(d) != (5 if model == 'plumb_bob' else 4)
                or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or min(k[0,0], k[1,1]) <= 0
                or not np.allclose(k[2], [0., 0., 1.]) or not np.allclose([k[0,1], k[1,0]], 0)):
            raise ValueError('Unsupported chest camera calibration')
        if not (0 <= k[0,2] < self.width and 0 <= k[1,2] < self.height):
            raise ValueError('Principal point lies outside the image')
        self.output_k = k.copy()
        self.output_k[0,0] *= focal_scale
        self.output_k[1,1] *= focal_scale
        size = (self.width, self.height)
        init = cv2.fisheye.initUndistortRectifyMap if model == 'equidistant' else cv2.initUndistortRectifyMap
        mx, my = init(k, d, np.eye(3), self.output_k, size, cv2.CV_32FC1)
        y, x = np.ogrid[:self.height, :self.width]
        radius_sq = ((x - k[0,2])/self.output_k[0,0])**2 + ((y - k[1,2])/self.output_k[1,1])**2
        self.radius_limit = _usable_radius(model, d)
        self.valid = (radius_sq < self.radius_limit**2) & np.isfinite(mx) & np.isfinite(my)
        # Ignore float round-off at the outside row and column of an identity map.
        self.valid &= (mx >= -1e-4) & (mx <= self.width-1+1e-4) & (my >= -1e-4) & (my <= self.height-1+1e-4)
        mx[~self.valid] = my[~self.valid] = -1.
        self.map_x, self.map_y = cv2.convertMaps(mx, my, cv2.CV_16SC2)

    def rectify(self, image):
        if image is None or image.shape[:2] != (self.height, self.width):
            raise ValueError('Image dimensions differ from chest calibration')
        return cv2.remap(image, self.map_x, self.map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def _chest_pair_matches(image, info):
    return (image.header.frame_id.lstrip('/') == info.header.frame_id.lstrip('/') == 'chest_usb_cam'
            and image.header.stamp == info.header.stamp and image.header.stamp.to_nsec() > 0
            and (info.width, info.height) == (image.width, image.height)
            and info.binning_x in (0, 1) and info.binning_y in (0, 1)
            and not (info.roi.x_offset or info.roi.y_offset)
            and info.roi.width in (0, info.width) and info.roi.height in (0, info.height))


def _capture(directory):
    from threading import Condition
    from time import monotonic
    import message_filters
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image
    rospy.init_node('r3_chest_board_capture', anonymous=True)
    directory.mkdir(parents=True, exist_ok=True)
    camera_file = directory / 'camera_info.json'
    bridge = CvBridge()
    condition = Condition()
    pair = None
    def receive(image, info):
        nonlocal pair
        if _chest_pair_matches(image, info):
            with condition:
                pair = (monotonic(), image, info)
                condition.notify()

    subscribers = [message_filters.Subscriber('/usb_cam_chest/image_raw', Image, queue_size=1, buff_size=2**22),
                   message_filters.Subscriber('/usb_cam_chest/camera_info', CameraInfo, queue_size=2)]
    sync = message_filters.TimeSynchronizer(subscribers, 4)
    sync.registerCallback(receive)
    while not rospy.is_shutdown():
        try:
            command = input('Move board, hold still and press Enter to capture (q to finish): ')
        except EOFError:
            break
        if command.strip().lower() == 'q':
            break
        entered = monotonic()
        with condition:
            if not condition.wait_for(lambda: pair is not None and pair[0] > entered, timeout=4.):
                print('No new synchronized chest frame. Try again.')
                continue
            _, image, info = pair
        calibration = dict(width=info.width, height=info.height, K=list(info.K), D=list(info.D),
                           R=list(info.R), P=list(info.P), distortion_model=info.distortion_model,
                           frame_id=info.header.frame_id)
        if camera_file.exists():
            if json.loads(camera_file.read_bytes()) != calibration:
                raise ValueError('Camera calibration changed; start a new capture directory')
        else:
            with camera_file.open('x', encoding='utf-8') as stream:
                json.dump(calibration, stream, indent=2)
        destination = directory / ('chest_%d.png' % image.header.stamp.to_nsec())
        ok, png = cv2.imencode('.png', bridge.imgmsg_to_cv2(image, desired_encoding='bgr8'))
        if not ok:
            raise ValueError('Could not encode raw chest image')
        try:
            with destination.open('xb') as stream:
                stream.write(png.tobytes())
        except FileExistsError:
            print('Frame timestamp already saved. Try again.')
            continue
        print(destination)


def _source_fingerprint(info):
    key = [int(info['width']), int(info['height']), info['distortion_model'],
           [float(v) for v in info['K']], [float(v) for v in info['D']]]
    return hashlib.sha256(json.dumps(key, separators=(',', ':'), allow_nan=False).encode('ascii')).hexdigest()


def _ros(profile=None):
    import message_filters
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import CameraInfo, Image
    rospy.init_node('r3_chest_rectification', anonymous=True)
    out = rospy.Publisher('/r3/chest_rect/image_rect', Image, queue_size=1)
    camera = rospy.Publisher('/r3/chest_rect/camera_info', CameraInfo, queue_size=1)
    src = message_filters.Subscriber('/usb_cam_chest/image_raw', Image, queue_size=1, buff_size=2**22)
    calibration = message_filters.Subscriber('/usb_cam_chest/camera_info', CameraInfo, queue_size=2)
    sync = message_filters.TimeSynchronizer([src, calibration], 4)
    bridge = CvBridge()
    mapping = key = None

    def publish(image, info):
        nonlocal mapping, key
        if not (out.get_num_connections() or camera.get_num_connections()):
            return
        try:
            if not _chest_pair_matches(image, info):
                raise ValueError('Expected synchronized full-resolution chest image and calibration')
            signature = info.width, info.height, info.distortion_model, tuple(info.K), tuple(info.D)
            if key != signature:
                measured = dict(width=info.width, height=info.height, K=info.K,
                                D=info.D, distortion_model=info.distortion_model)
                selected = measured
                if profile is not None:
                    if (profile.get('source_calibration_sha256') == _source_fingerprint(measured)
                            and profile.get('frame_id') == info.header.frame_id.lstrip('/')
                            and (profile.get('width'), profile.get('height')) == (info.width, info.height)):
                        selected = profile
                    else:
                        rospy.logwarn('Chest profile differs from live camera; using its current CameraInfo')
                mapping = ChestRectification(selected)
                key = signature
                rospy.loginfo('Chest rectification: %.1f%% of output has supported rays', 100*mapping.valid.mean())
            rect = mapping.rectify(bridge.imgmsg_to_cv2(image, desired_encoding='bgr8'))
            result = bridge.cv2_to_imgmsg(rect, encoding='bgr8')
            result.header = image.header
            calibrated = CameraInfo()
            calibrated.header = image.header
            calibrated.width, calibrated.height = info.width, info.height
            calibrated.distortion_model, calibrated.D = 'plumb_bob', [0.0]*5
            calibrated.K = mapping.output_k.ravel().tolist()
            calibrated.R = np.eye(3).ravel().tolist()
            calibrated.P = np.column_stack((mapping.output_k, np.zeros(3))).ravel().tolist()
            camera.publish(calibrated)
            out.publish(result)
        except (ValueError, cv2.error, CvBridgeError) as exc:
            rospy.logwarn_throttle(3., 'Skipping chest rectification frame: %s', str(exc))

    sync.registerCallback(publish)
    rospy.spin()


def main():
    parser = argparse.ArgumentParser(description='Rectify the chest image using its measured intrinsic calibration')
    sub = parser.add_subparsers(dest='mode', required=True)
    offline = sub.add_parser('image', help='Rectify a raw PNG or JPEG offline')
    offline.add_argument('calibration', type=Path)
    offline.add_argument('source', type=Path)
    offline.add_argument('output', type=Path)
    offline.add_argument('--mask', type=Path, help='Write the supported output region as a grayscale image')
    offline.add_argument('--focal-scale', type=float, default=1., help='Virtual focal length divided by input focal length')
    capture = sub.add_parser('capture', help='Save original chest PNG files from successive stationary board poses')
    capture.add_argument('directory', type=Path)
    ros = sub.add_parser('ros', help='Publish a separate rectified chest image with matching CameraInfo')
    ros.add_argument('--calibration', type=Path, help='Use a field profile matching the live source camera')
    opts = parser.parse_args()
    if opts.mode == 'ros':
        profile = None
        if opts.calibration:
            profile = json.loads(opts.calibration.read_text(encoding='utf-8-sig'))
            ChestRectification(profile)
        return _ros(profile)
    if opts.mode == 'capture':
        return _capture(opts.directory)
    data = json.loads(opts.calibration.read_text(encoding='utf-8-sig'))
    info = dict(data.get('ci', data))
    info.setdefault('width', info.get('w'))
    info.setdefault('height', info.get('h'))
    files = [opts.source, opts.calibration, opts.output] + ([opts.mask] if opts.mask else [])
    if len({p.resolve() for p in files}) != len(files):
        parser.error('Use distinct paths for inputs, output and mask')
    mapping = ChestRectification(info, opts.focal_scale)
    image = cv2.imdecode(np.fromfile(str(opts.source), np.uint8), cv2.IMREAD_UNCHANGED)
    rect = mapping.rectify(image)
    outputs = [(opts.output, rect)]
    if opts.mask:
        outputs.append((opts.mask, mapping.valid.astype(np.uint8)*255))
    for path, pixels in outputs:
        ok, encoded = cv2.imencode(path.suffix, pixels)
        if not ok:
            raise ValueError('Could not encode '+str(path))
        path.write_bytes(encoded.tobytes())
    print('Supported output %.2f%% (%dx%d)' % (mapping.valid.mean()*100, mapping.width, mapping.height))


if __name__ == '__main__':
    main()
