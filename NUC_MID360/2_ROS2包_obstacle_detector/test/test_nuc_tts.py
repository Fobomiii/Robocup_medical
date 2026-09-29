"""NUC speech protocol and offline player tests."""

import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from obstacle_detector.nav_protocol import (
    FrameParser,
    MSG_TTS_REQUEST,
    MSG_TTS_STATUS,
    TTS_STATUS_COMPLETED,
    TtsRequest,
    TtsStatus,
    decode_tts_request,
    decode_tts_status,
    encode_frame,
    encode_tts_request,
    encode_tts_status,
)
from obstacle_detector.nuc_tts import NucTtsPlayer


class NucTtsProtocolTest(unittest.TestCase):
    def test_request_and_status_round_trip(self):
        request_frame = FrameParser().feed(
            encode_frame(MSG_TTS_REQUEST, 7, encode_tts_request(513, 3))
        )[0]
        self.assertEqual(decode_tts_request(request_frame.payload), TtsRequest(513, 3))

        status_frame = FrameParser().feed(
            encode_frame(
                MSG_TTS_STATUS,
                8,
                encode_tts_status(513, TTS_STATUS_COMPLETED),
            )
        )[0]
        self.assertEqual(
            decode_tts_status(status_frame.payload),
            TtsStatus(513, TTS_STATUS_COMPLETED),
        )

    def test_invalid_bed_and_status_are_rejected(self):
        with self.assertRaises(ValueError):
            encode_tts_request(1, 2)
        with self.assertRaises(ValueError):
            decode_tts_request(b"\x00\x01\x02")
        with self.assertRaises(ValueError):
            encode_tts_status(1, 0)
        with self.assertRaises(ValueError):
            decode_tts_status(b"\x00\x01\x03")


class NucTtsPlayerTest(unittest.TestCase):
    def test_keepalive_holds_a_silent_pulse_stream_until_stopped(self):
        process = Mock()
        process.poll.return_value = None
        popen = Mock(return_value=process)
        player = NucTtsPlayer(audio_device="pulse", popen_command=popen)

        result = player.start_keepalive()

        self.assertTrue(result.success)
        command = popen.call_args.args[0]
        self.assertEqual(command[:4], ["aplay", "-q", "-D", "pulse"])
        self.assertEqual(command[-1], "/dev/zero")
        self.assertTrue(player.keepalive_active)

        player.stop_keepalive()

        process.terminate.assert_called_once_with()
        process.wait.assert_called_once_with(timeout=1.0)

    def test_audio_environment_targets_service_users_pulse_socket(self):
        with patch.dict("os.environ", {}, clear=True), patch(
            "obstacle_detector.nuc_tts.os.getuid", create=True, return_value=1000
        ), patch(
            "obstacle_detector.nuc_tts.os.path.isdir", return_value=True
        ), patch(
            "obstacle_detector.nuc_tts.os.path.exists", return_value=True
        ):
            environment = NucTtsPlayer._audio_environment()

        self.assertEqual(environment["XDG_RUNTIME_DIR"], "/run/user/1000")
        self.assertEqual(
            environment["PULSE_SERVER"], "unix:/run/user/1000/pulse/native"
        )

    def test_bed_mp3_is_decoded_then_played_at_maximum_volume(self):
        mixer_result = subprocess.CompletedProcess([], 0, b"", b"")
        decode_result = subprocess.CompletedProcess([], 0, b"RIFF-test", b"")
        playback_result = subprocess.CompletedProcess([], 0, b"", b"")
        runner = Mock(
            side_effect=[
                mixer_result,
                mixer_result,
                mixer_result,
                decode_result,
                playback_result,
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            bed1_audio = Path(directory) / "1_.mp3"
            bed3_audio = Path(directory) / "3_.mp3"
            bed1_audio.write_bytes(b"test")
            bed3_audio.write_bytes(b"test")
            player = NucTtsPlayer(
                bed1_audio_file=str(bed1_audio),
                bed3_audio_file=str(bed3_audio),
                run_command=runner,
            )

            result = player.speak_bed(1)

        self.assertTrue(result.success)
        decode_command = runner.call_args_list[3].args[0]
        self.assertEqual(decode_command[0], "ffmpeg")
        self.assertIn(str(bed1_audio), decode_command)
        self.assertNotIn("-af", decode_command)
        self.assertEqual(
            runner.call_args_list[4].args[0],
            ["aplay", "-q", "-D", "default"],
        )
        self.assertEqual(runner.call_args_list[4].kwargs["input"], b"RIFF-test")

    def test_missing_audio_file_reports_failure_without_starting_player(self):
        runner = Mock()
        player = NucTtsPlayer(
            bed1_audio_file="/missing/1_.mp3",
            bed3_audio_file="/missing/3_.mp3",
            run_command=runner,
        )

        result = player.speak_bed(3)

        self.assertFalse(result.success)
        self.assertIn("not readable", result.detail)
        runner.assert_not_called()


if __name__ == "__main__":
    unittest.main()
