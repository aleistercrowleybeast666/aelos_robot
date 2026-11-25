import threading
import unittest
from unittest.mock import Mock, patch

from r3_deployer import chest_camera as camera


def packet(payload, stamp=1):
    jpeg = b'\xff\xd8'+payload+b'\xff\xd9'
    return camera._HEADER.pack(camera._FRAME_MAGIC, len(jpeg), stamp, 640, 480)+jpeg


class CameraStreamTests(unittest.TestCase):
    def test_decoder_keeps_newest_complete_frame_across_chunks(self):
        stream = camera._FrameDecoder()
        first, second, third = packet(b'a'), packet(b'b', 2), packet(b'c', 3)
        self.assertIsNone(stream.feed(first[:5]))
        newest = stream.feed(first[5:]+second+third[:25])
        self.assertEqual((newest.jpeg, newest.timestamp), (b'\xff\xd8b\xff\xd9', 2))
        self.assertEqual(stream.feed(third[25:]).timestamp, 3)

    def test_decoder_rejects_unbounded_frame_header(self):
        stream = camera._FrameDecoder()
        header = camera._HEADER.pack(camera._FRAME_MAGIC, camera._MAX_JPEG_BYTES+1, 1, 640, 480)
        with self.assertRaisesRegex(RuntimeError, '无效的图像帧头'):
            stream.feed(header)

    def test_no_frame_produces_camera_specific_diagnostic(self):
        channel = Mock()
        channel.recv_ready.return_value = False
        channel.recv_stderr_ready.return_value = False
        channel.exit_status_ready.return_value = False
        with patch.object(camera.time, 'monotonic', side_effect=[0, camera._FIRST_FRAME_TIMEOUT+1]):
            with self.assertRaisesRegex(RuntimeError, '/usb_cam_chest/image_raw'):
                next(camera._read_frames(channel, threading.Event()))

    def test_explicit_stop_ends_preview_and_closes_independent_channel(self):
        channel = Mock()
        channel.recv_ready.side_effect = [True, False]
        channel.recv.return_value = packet(b'frame')
        link = Mock()
        link.__enter__ = Mock(return_value=link)
        link.__exit__ = Mock(return_value=False)
        link.get_transport.return_value.open_session.return_value = channel
        link.get_transport.return_value.is_active.return_value = True
        stop = threading.Event()
        with patch('paramiko.SSHClient', return_value=link), patch.object(camera, '_connect'):
            frames = camera.ChestCamera(camera.settings.DEFAULTS, password='fixture').frames(stop)
            self.assertEqual(next(frames).width, 640)
            stop.set()
            with self.assertRaises(StopIteration):
                next(frames)
        channel.close.assert_called_once()
        link.__exit__.assert_called_once()

    def test_remote_worker_compiles_and_selects_existing_chest_topic(self):
        compile(camera._PI_SUBSCRIBER, '<Pi subscriber>', 'exec')
        command = camera._command()
        self.assertIn('/usb_cam_chest/image_raw', command)
        self.assertIn(camera.DEVICE_IDENTITY_SHA256, command)
        self.assertNotIn('CMDcontrol', command)


if __name__ == '__main__':
    unittest.main()
