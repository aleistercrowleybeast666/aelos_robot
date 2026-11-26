"""Emit an offline A/B patch for the confirmed AELOS ALVAR source.

Run with the vendor ar_track_alvar package directory as the sole argument.
The unified diff is written to stdout in UTF-8. Apply it to a separate source
checkout and build there. The original files and ROS services are never opened
for writing. Replay the same raw images, CameraInfo and TF on an isolated ROS
master with the stock node, then the /r3_optics_trial/ar_track_alvar node
with its private use_raw_chest_distortion parameter set to true before launch.
The stock node also broadcasts TF, so use separate replay runs.
"""

import argparse
import difflib
import hashlib
from pathlib import Path
import sys


_CAMERA_HASH = '0691938547b23708973e6f3e2755db4ca7842c6f13b0e46fd183d50c0bf3955c'
_NODE_HASH = 'ca410bc576e6eed551a682bc1f326fd15ae9bc8e35058e48a4d28e1676dbd421'
_ORIGINAL = '  cv::solvePnP(pw, pi2, calib_K, cv::Mat(), rodriques, tra, false,\n'
_SOLVER = 'void Camera::CalcExteriorOrientation(const vector<cv::Point3d>& pw,'
_CORNER_CHECK = '''// Only accept the optical-axis branch where the Brown-Conrady map is
// locally invertible and its inverse maps all four measured corners back.
static bool Chest_RawCornersMatchCalibration(const vector<cv::Point2d>& pixels,
                                            const cv::Mat& K, const vector<double>& d)
{
  if (pixels.size() != 4)
    return false;
  vector<cv::Point2d> normalized, reprojected;
  vector<cv::Point3d> rays;
  cv::undistortPoints(pixels, normalized, K, d);
  for (size_t i = 0; i < pixels.size(); ++i)
  {
    if (!(pixels[i].x >= 0 && pixels[i].x < 640 && pixels[i].y >= 0 && pixels[i].y < 480))
      return false;
    rays.emplace_back(normalized[i].x, normalized[i].y, 1.);
    for (int quarter = 1; quarter <= 4; ++quarter)
    {
      const double x = normalized[i].x * quarter / 4., y = normalized[i].y * quarter / 4.;
      const double r2 = x*x + y*y;
      const double radial = 1. + r2 * (d[0] + r2 * (d[1] + r2 * d[4]));
      const double slope = 2. * (d[0] + r2 * (2. * d[1] + 3. * r2 * d[4]));
      const double xx = radial + x*x*slope + 2.*d[2]*y + 6.*d[3]*x;
      const double yy = radial + y*y*slope + 6.*d[2]*y + 2.*d[3]*x;
      const double xy = x*y*slope + 2.*d[2]*x + 2.*d[3]*y;
      // Require both eigenvalues of the symmetric model Jacobian to exceed 0.2.
      if (!(xx > .2 && yy > .2 && (xx-.2)*(yy-.2) > xy*xy))
        return false;
    }
  }
  cv::projectPoints(rays, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), K, d, reprojected);
  for (size_t i = 0; i < pixels.size(); ++i)
    if (!(cv::norm(pixels[i] - reprojected[i]) <= .5))
      return false;
  return true;
}

'''
_REPLACEMENT = '''  cv::Mat raw_distortion;
  const std::string& node = ros::this_node::getName();
  if (!cameraInfoTopic_.empty() &&
      (node == "/r3_optics_trial/ar_track_alvar" || node == "/chest/ar_track_alvar"))
  {
    // One setting per node process. Changing the model requires restarting this detector.
    static const bool enabled = ros::param::param<bool>(node + "/use_raw_chest_distortion", false);
    // This node consumes the vendor detector's original, distorted corner pixels.
    const sensor_msgs::CameraInfo& info = cam_info_;
    const bool raw_topics = cameraInfoTopic_ == "camera_info" &&
        n_.resolveName("camera_info") == "/usb_cam_chest/camera_info" &&
        n_.resolveName("camera_image") == "/usb_cam_chest/image_raw";
    const bool full_frame = info.width == 640 && info.height == 480 &&
        x_res == 640 && y_res == 480 && info.binning_x <= 1 && info.binning_y <= 1 &&
        info.roi.x_offset == 0 && info.roi.y_offset == 0 &&
        info.roi.width == 0 && info.roi.height == 0;
    if (enabled && raw_topics && full_frame &&
        (info.header.frame_id == "chest_usb_cam" || info.header.frame_id == "/chest_usb_cam") &&
        info.distortion_model == "plumb_bob" && info.D.size() == 5 &&
        calib_K.at<double>(0, 0) > 0 && calib_K.at<double>(1, 1) > 0 &&
        cv::checkRange(calib_K) && cv::checkRange(cv::Mat(info.D)))
    {
      // SetCameraInfo keeps the full ROS message, including k3.
      if (Chest_RawCornersMatchCalibration(pi2, calib_K, info.D))
      {
        raw_distortion = cv::Mat(info.D);
        ROS_INFO_ONCE("R3: using chest CameraInfo K and D for raw-corner PnP");
      }
      else
      {
        ROS_WARN_THROTTLE(2., "R3: raw corner inverse check failed, using stock PnP for this marker");
      }
    }
    else if (enabled)
    {
      ROS_WARN_ONCE("R3: chest raw calibration contract unavailable, using stock PnP");
    }
  }
  cv::solvePnP(pw, pi2, calib_K, raw_distortion, rodriques, tra, false,
'''


def _read_verified(path, expected_hash):
    payload = path.read_bytes()
    if hashlib.sha256(payload).hexdigest() != expected_hash:
        raise ValueError(f'{path.name} differs from the verified AELOS source')
    return payload.decode('utf-8')


def Alvar_BuildChestOpticsPatch(package):
    _read_verified(package / 'nodes' / 'IndividualMarkersNoKinect.cpp', _NODE_HASH)
    source = _read_verified(package / 'src' / 'Camera.cpp', _CAMERA_HASH)
    if source.count(_ORIGINAL) != 1 or source.count(_SOLVER) != 1:
        raise ValueError('Expected one shared OpenCV pose solver')
    modified = source.replace(_SOLVER, _CORNER_CHECK + _SOLVER).replace(_ORIGINAL, _REPLACEMENT)
    return ''.join(difflib.unified_diff(source.splitlines(keepends=True), modified.splitlines(keepends=True),
                                        fromfile='a/src/Camera.cpp', tofile='b/src/Camera.cpp'))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('package', type=Path, help='vendor ar_track_alvar package containing nodes/ and src/')
    parser.add_argument('--output', type=Path, help='write a new patch file instead of stdout')
    args = parser.parse_args()
    try:
        patch = Alvar_BuildChestOpticsPatch(args.package).encode('utf-8')
        if args.output:
            with args.output.open('xb') as output:
                output.write(patch)
        else:
            sys.stdout.buffer.write(patch)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    main()
