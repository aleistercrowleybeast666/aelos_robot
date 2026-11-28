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

    def test_preview_reopens_after_a_gap_or_link_loss_but_not_a_protocol_error(self):
        stop = threading.Event()
        retry = Mock()
        def stalled():
            yield 1
            raise camera.PreviewInterrupted('gap')
        source = Mock(side_effect=[stalled(), camera.SshLinkError('link'), iter([2])])
        with patch.object(camera, '_RECONNECT_DELAYS', (0.,) * 3):
            self.assertEqual(list(camera.preview_frames(source,stop,retry)), [1,2])
        self.assertEqual([call.args for call in retry.call_args_list], [(1,3),(2,3)])
        source = Mock(side_effect=RuntimeError('wrong identity'))
        with self.assertRaisesRegex(RuntimeError, 'wrong identity'):
            list(camera.preview_frames(source,stop,retry))
        source.assert_called_once()

    def test_flapping_stream_exhausts_retries_and_a_stable_stream_restores_them(self):
        def short():
            yield 1
            raise camera.PreviewInterrupted('gap')
        def stable():
            yield 1
            yield 2
            raise camera.PreviewInterrupted('gap')
        for third_stream, clock, recovers in (
                (short, [0.,1.,2.], False), (stable, [0.,1.,2.,25.,26.], True)):
            with self.subTest(recovers=recovers), patch.object(camera,'_RECONNECT_DELAYS',(0.,0.)), \
                    patch.object(camera.time,'monotonic',side_effect=clock):
                source = Mock(side_effect=[short(), short(), third_stream(), iter([3])])
                retry = Mock()
                stream = camera.preview_frames(source, threading.Event(), retry)
                if recovers:
                    self.assertEqual(list(stream), [1,1,1,2,3])
                    self.assertEqual(retry.call_args.args, (1,2))
                else:
                    with self.assertRaises(camera.PreviewInterrupted):
                        list(stream)
                    self.assertEqual(source.call_count, 3)

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
