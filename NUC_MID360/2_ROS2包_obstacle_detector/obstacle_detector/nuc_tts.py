"""Offline NUC playback for pre-generated bedside announcements."""

from dataclasses import dataclass
import os
import subprocess
from typing import Callable, Dict


DEFAULT_BED_AUDIO_FILES: Dict[int, str] = {
    1: "/home/fzurobot/Downloads/1_.mp3",
    3: "/home/fzurobot/Downloads/3_.mp3",
}


@dataclass(frozen=True)
class TtsPlaybackResult:
    success: bool
    detail: str


class NucTtsPlayer:
    def __init__(
        self,
        bed1_audio_file: str = DEFAULT_BED_AUDIO_FILES[1],
        bed3_audio_file: str = DEFAULT_BED_AUDIO_FILES[3],
        volume_percent: int = 100,
        audio_device: str = "default",
        lead_silence_s: float = 0.0,
        tail_silence_s: float = 0.0,
        keepalive_enabled: bool = True,
        timeout_s: float = 8.0,
        run_command: Callable = subprocess.run,
        popen_command: Callable = subprocess.Popen,
    ) -> None:
        self.audio_files = {
            1: os.path.abspath(os.path.expanduser(bed1_audio_file)),
            3: os.path.abspath(os.path.expanduser(bed3_audio_file)),
        }
        self.volume_percent = max(0, min(100, int(volume_percent)))
        self.audio_device = audio_device
        self.lead_silence_ms = max(0, round(float(lead_silence_s) * 1000.0))
        self.tail_silence_s = max(0.0, float(tail_silence_s))
        self.keepalive_enabled = bool(keepalive_enabled)
        self.timeout_s = max(1.0, float(timeout_s))
        self._run_command = run_command
        self._popen_command = popen_command
        self._keepalive_process = None

    @staticmethod
    def _audio_environment() -> Dict[str, str]:
        environment = os.environ.copy()
        runtime_dir = environment.get("XDG_RUNTIME_DIR")
        getuid = getattr(os, "getuid", None)
        if not runtime_dir and getuid is not None:
            runtime_dir = f"/run/user/{getuid()}"
        if not runtime_dir:
            return environment
        if os.path.isdir(runtime_dir):
            environment["XDG_RUNTIME_DIR"] = runtime_dir
            normalized_runtime_dir = runtime_dir.replace("\\", "/").rstrip("/")
            pulse_socket = f"{normalized_runtime_dir}/pulse/native"
            if os.path.exists(pulse_socket):
                environment.setdefault("PULSE_SERVER", f"unix:{pulse_socket}")
        return environment

    def _set_volume(self) -> None:
        volume = f"{self.volume_percent}%"
        environment = self._audio_environment()
        for control in ("Master", "Speaker", "PCM"):
            try:
                self._run_command(
                    ["amixer", "-q", "sset", control, volume, "unmute"],
                    env=environment,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2.0,
                    check=False,
                )
            except (OSError, subprocess.SubprocessError):
                pass

    @property
    def keepalive_active(self) -> bool:
        return (
            self._keepalive_process is not None
            and self._keepalive_process.poll() is None
        )

    def start_keepalive(self) -> TtsPlaybackResult:
        if not self.keepalive_enabled:
            return TtsPlaybackResult(True, "audio keepalive disabled")
        if self.keepalive_active:
            return TtsPlaybackResult(
                True, f"audio keepalive active via {self.audio_device}"
            )

        self.stop_keepalive()
        try:
            self._keepalive_process = self._popen_command(
                [
                    "aplay",
                    "-q",
                    "-D",
                    self.audio_device,
                    "-t",
                    "raw",
                    "-f",
                    "S16_LE",
                    "-r",
                    "48000",
                    "-c",
                    "2",
                    "/dev/zero",
                ],
                env=self._audio_environment(),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            self._keepalive_process = None
            return TtsPlaybackResult(False, str(exc))
        return TtsPlaybackResult(
            True, f"audio keepalive started via {self.audio_device}"
        )

    def stop_keepalive(self) -> None:
        process = self._keepalive_process
        self._keepalive_process = None
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def speak_bed(self, bed: int) -> TtsPlaybackResult:
        audio_file = self.audio_files.get(int(bed))
        if audio_file is None:
            return TtsPlaybackResult(False, f"unsupported bed {bed}")
        if not os.path.isfile(audio_file) or not os.access(audio_file, os.R_OK):
            return TtsPlaybackResult(False, f"audio file is not readable: {audio_file}")

        self.start_keepalive()
        self._set_volume()
        environment = self._audio_environment()
        decode_command = [
            "ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            audio_file,
        ]
        audio_filters = []
        if self.lead_silence_ms > 0:
            audio_filters.append(f"adelay={self.lead_silence_ms}:all=1")
        if self.tail_silence_s > 0.0:
            audio_filters.append(f"apad=pad_dur={self.tail_silence_s:.3f}")
        if audio_filters:
            decode_command.extend(["-af", ",".join(audio_filters)])
        decode_command.extend(["-f", "wav", "-"])
        try:
            decoding = self._run_command(
                decode_command,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.timeout_s,
                check=False,
            )
            if decoding.returncode != 0 or not decoding.stdout:
                detail = decoding.stderr.decode("utf-8", "replace").strip()
                return TtsPlaybackResult(
                    False, detail or f"ffmpeg exited {decoding.returncode}"
                )

            playback_errors = []
            devices = [self.audio_device]
            if self.audio_device != "default":
                devices.append("default")
            for device in devices:
                playback = self._run_command(
                    ["aplay", "-q", "-D", device],
                    input=decoding.stdout,
                    env=environment,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    timeout=self.timeout_s,
                    check=False,
                )
                if playback.returncode == 0:
                    return TtsPlaybackResult(True, f"{audio_file} via {device}")
                detail = playback.stderr.decode("utf-8", "replace").strip()
                playback_errors.append(
                    f"{device}: {detail or f'aplay exited {playback.returncode}'}"
                )
            return TtsPlaybackResult(False, "; ".join(playback_errors))
        except FileNotFoundError as exc:
            return TtsPlaybackResult(False, f"missing command: {exc.filename}")
        except subprocess.TimeoutExpired:
            return TtsPlaybackResult(False, "speech playback timed out")
        except OSError as exc:
            return TtsPlaybackResult(False, str(exc))

        return TtsPlaybackResult(False, "speech playback ended unexpectedly")
