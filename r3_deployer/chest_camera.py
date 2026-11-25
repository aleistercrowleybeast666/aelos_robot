"""Read the R3 chest camera over an independent, verified SSH connection."""

from dataclasses import dataclass
import math
import shlex
import struct
import time

from . import settings
from .deploy import SshLinkError, _connect
from .remote import DEVICE_IDENTITY_SHA256


_TOPIC = '/usb_cam_chest/image_raw'
_MAX_WIDTH = 640
_MAX_HEIGHT = 480
_FPS = 7
_MAX_JPEG_BYTES = 1024 * 1024
_HEADER = struct.Struct('!4sIdII')
_FRAME_MAGIC = b'R3CF'
_FIRST_FRAME_TIMEOUT = 15
_FRAME_GAP_TIMEOUT = 5
_POLL_SECONDS = 0.05

# This subscriber runs on the Pi. Keep frame data on a dedicated stdout
# descriptor so output from ROS or native libraries cannot corrupt the stream.
_PI_SUBSCRIBER = r"""
from collections import deque
import hashlib
import math
import os
from pathlib import Path
import re
import struct
import sys
import threading
import time

identity_sha256, topic, max_width, max_height, fps = sys.argv[1:]
max_width, max_height, fps = int(max_width), int(max_height), int(fps)
cpu = re.search(r'^Serial\s*:\s*(\S+)', Path('/proc/cpuinfo').read_text(), re.M)
machine_id = Path('/etc/machine-id').read_text().strip()
if not cpu or hashlib.sha256((cpu[1] + '\0' + machine_id).encode('utf-8')).hexdigest() != identity_sha256:
    sys.exit('R3 CPU / machine identity mismatch')

stream = os.fdopen(os.dup(sys.stdout.fileno()), 'wb')
os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
stopped = threading.Event()
finished = threading.Event()

def watch_stdin():
    # Closing the SSH channel shuts down the subscription even if the camera
    # is idle. Give ROS time to unregister before forcing a blocked process out.
    try:
        sys.stdin.buffer.read(1)
    finally:
        stopped.set()
        if not finished.wait(0.8):
            os._exit(0)

threading.Thread(target=watch_stdin, daemon=True).start()
try:
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    cv2.setNumThreads(1)
    rospy.init_node('r3_pc_chest_preview', anonymous=True,
                    disable_signals=True, disable_rosout=True, disable_rostime=True)
    newest = deque(maxlen=1)
    bridge = CvBridge()
    sub = rospy.Subscriber(topic, Image, newest.append,
                           queue_size=1, buff_size=8 * 1024 * 1024)
    try:
        while not stopped.wait(1 / fps) and not rospy.is_shutdown():
            try:
                source = newest.popleft()
            except IndexError:
                continue
            image = bridge.imgmsg_to_cv2(source, desired_encoding='bgr8')
            height, width = image.shape[:2]
            ratio = min(1, max_width / width, max_height / height)
            if ratio < 1:
                size = (max(1, int(width * ratio)), max(1, int(height * ratio)))
                image = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
            height, width = image.shape[:2]
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 73])
            if not ok:
                continue
            jpeg = encoded.tobytes()
            timestamp = source.header.stamp.to_sec()
            if not math.isfinite(timestamp) or timestamp <= 0:
                timestamp = time.time()
            stream.write(struct.pack('!4sIdII', b'R3CF', len(jpeg), timestamp, width, height))
            stream.write(jpeg)
            stream.flush()
    finally:
        sub.unregister()
        rospy.signal_shutdown('chest preview closed')
except BrokenPipeError:
    pass
finally:
    stream.close()
    finished.set()
"""


@dataclass(frozen=True)
class CameraFrame:
    """A JPEG preview frame with capture time and encoded dimensions."""

    jpeg: bytes
    width: int
    height: int
    timestamp: float


class _FrameDecoder:
    def __init__(self):
        self._pending = bytearray()

    def feed(self, data):
        self._pending.extend(data)
        newest = None
        while len(self._pending) >= _HEADER.size:
            magic, size, timestamp, width, height = _HEADER.unpack_from(self._pending)
            if (magic != _FRAME_MAGIC or not 4 <= size <= _MAX_JPEG_BYTES
                    or not 0 < width <= _MAX_WIDTH or not 0 < height <= _MAX_HEIGHT
                    or not math.isfinite(timestamp) or timestamp <= 0):
                raise RuntimeError('胸部相机返回了无效的图像帧头。')
            end = _HEADER.size + size
            if len(self._pending) < end:
                break
            jpeg = bytes(self._pending[_HEADER.size:end])
            del self._pending[:end]
            if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                raise RuntimeError('胸部相机返回了不完整的 JPEG。')
            newest = CameraFrame(jpeg, width, height, timestamp)
        return newest


def _command():
    setup = ('for setup in /opt/ros/noetic/setup.bash '
             '"$HOME/ros_catkin_ws/install_isolated/setup.bash" '
             '"$HOME/catkin_ws/devel/setup.bash"; do '
             'if test -r "$setup"; then . "$setup" 1>&2; fi; done; ')
    args = ('python3', '-B', '-u', '-c', _PI_SUBSCRIBER, DEVICE_IDENTITY_SHA256,
            _TOPIC, str(_MAX_WIDTH), str(_MAX_HEIGHT), str(_FPS))
    return 'bash -c ' + shlex.quote(setup + 'exec ' + shlex.join(args))


def _read_frames(channel, stop_event):
    decoder = _FrameDecoder()
    stderr = bytearray()
    last_frame = None
    started = time.monotonic()
    while not stop_event.is_set():
        # Drain the frames already buffered by SSH and present only the newest
        # complete one to the UI. Bound work per poll to keep cancellation responsive.
        newest = None
        for _ in range(16):
            if stop_event.is_set() or not channel.recv_ready():
                break
            chunk = channel.recv(65536)
            if not chunk:
                break
            newest = decoder.feed(chunk) or newest
        if stop_event.is_set():
            return
        if newest is not None:
            last_frame = time.monotonic()
            yield newest
        if stop_event.is_set():
            return
        for _ in range(4):
            if not channel.recv_stderr_ready():
                break
            stderr.extend(channel.recv_stderr(4096))
            del stderr[:-4096]
        if channel.exit_status_ready() and not channel.recv_ready():
            error = stderr.decode('utf-8', 'replace').strip().splitlines()
            exit_code = channel.recv_exit_status()
            if exit_code < 0 and not error:
                raise SshLinkError('胸部相机的 SSH 通道已关闭。')
            detail = error[-1][:400] if error else f'退出码 {exit_code}'
            raise RuntimeError('胸部相机连接已结束。'+detail)
        now = time.monotonic()
        if last_frame is None and now - started > _FIRST_FRAME_TIMEOUT:
            raise RuntimeError(f'一直没有收到胸部相机画面，请检查 {_TOPIC} 和 USB 相机连接。')
        if last_frame is not None and now - last_frame > _FRAME_GAP_TIMEOUT:
            raise RuntimeError('胸部相机画面中断，请检查相机和无线连接后重新打开预览。')
        stop_event.wait(_POLL_SECONDS)


class ChestCamera:
    """Stream only the existing R3 chest Image topic. Uses no motion commands."""

    def __init__(self, values=None, password=None, log=None):
        self._values = settings.validate(settings.load() if values is None else values)
        self._password = password
        self._log = log

    def frames(self, stop_event):
        """Yield frames until stopped. Closing this iterator closes its SSH session."""
        import paramiko

        if stop_event.is_set():
            return
        password = self._password or settings.password_for(self._values)
        if not password:
            raise ValueError('请输入或保存 SSH 密码，再打开胸部相机。')
        try:
            with paramiko.SSHClient() as ssh:
                _connect(ssh, self._values, password)
                if stop_event.is_set():
                    return
                transport = ssh.get_transport()
                if transport is None or not transport.is_active():
                    raise SshLinkError('胸部相机的 SSH 连接未建立。')
                channel = transport.open_session(timeout=7)
                try:
                    if stop_event.is_set():
                        return
                    channel.exec_command(_command())
                    if self._log is not None:
                        self._log('正在等待胸部相机图像…')
                    yield from _read_frames(channel, stop_event)
                finally:
                    channel.close()
        except SshLinkError:
            if not stop_event.is_set():
                raise
        except (OSError, EOFError) as exc:
            if not stop_event.is_set():
                raise SshLinkError(str(exc).replace(password, '******')) from exc
        except (paramiko.SSHException, RuntimeError) as exc:
            if not stop_event.is_set():
                raise RuntimeError(str(exc).replace(password, '******')) from None
