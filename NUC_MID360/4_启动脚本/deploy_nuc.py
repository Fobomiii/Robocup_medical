#!/usr/bin/env python3
"""Deploy navigation, scanning and the fullscreen NUC dashboard."""

import os
import posixpath
import shlex
import stat
import sys
import time

import paramiko


HOST = os.environ.get("MEDICAL_NUC_HOST", "192.168.50.22")
USER = os.environ.get("MEDICAL_NUC_USER", "fzurobot")
PASSWORD = os.environ.get("MEDICAL_NUC_PASSWORD")
ROS_DOMAIN_ID = os.environ.get("MEDICAL_ROS_DOMAIN_ID", "77")
SCANNER_ENABLED = os.environ.get("MEDICAL_SCANNER_ENABLED", "true").lower()
SCAN_CAMERA = os.environ.get(
    "MEDICAL_SCAN_CAMERA",
    "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0",
)
TELE_SCAN_CAMERA = os.environ.get(
    "MEDICAL_TELE_SCAN_CAMERA",
    "/dev/v4l/by-id/usb-BLC-240823--A_SDYH-8P0P-video-index0",
)
HEALTH_BLE_DEVICE_NAME = os.environ.get(
    "MEDICAL_HEALTH_BLE_DEVICE_NAME", "MedicalVitals-S3"
)
HEALTH_BLE_DEVICE_ADDRESS = os.environ.get(
    "MEDICAL_HEALTH_BLE_DEVICE_ADDRESS", ""
)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "2_ROS2包_obstacle_detector")
PLANNER_PKG = os.path.join(ROOT, "2_ROS2包_clearance_planner")
VIZ = os.path.join(ROOT, "3_可视化工具")

def iter_package_files(remote_pkg: str, remote_planner_pkg: str):
    packages = (
        (PKG, remote_pkg),
        (PLANNER_PKG, remote_planner_pkg),
    )
    for local_root, remote_root in packages:
        for directory, names, files in os.walk(local_root):
            names[:] = [
                name for name in names
                if name not in {"__pycache__", ".pytest_cache", "build", "install", "log"}
            ]
            for filename in files:
                if filename.endswith((".pyc", ".pyo")):
                    continue
                local = os.path.join(directory, filename)
                relative = os.path.relpath(local, local_root).replace(os.sep, "/")
                yield local, posixpath.join(remote_root, relative)


def main() -> int:
    if not PASSWORD:
        print("Set MEDICAL_NUC_PASSWORD before running deployment.")
        return 2
    if not ROS_DOMAIN_ID.isdigit() or not 0 <= int(ROS_DOMAIN_ID) <= 101:
        print("MEDICAL_ROS_DOMAIN_ID must be an integer from 0 to 101.")
        return 2
    if SCANNER_ENABLED not in {"true", "false"}:
        print("MEDICAL_SCANNER_ENABLED must be true or false.")
        return 2
    if not SCAN_CAMERA.startswith("/dev/"):
        print("MEDICAL_SCAN_CAMERA must be an absolute /dev path.")
        return 2
    if not TELE_SCAN_CAMERA.startswith("/dev/"):
        print("MEDICAL_TELE_SCAN_CAMERA must be an absolute /dev path.")
        return 2
    for package_root in (PKG, PLANNER_PKG):
        manifest = os.path.join(package_root, "package.xml")
        if not os.path.isfile(manifest):
            print(f"missing ROS package manifest: {manifest}")
            return 1

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        HOST,
        username=USER,
        password=PASSWORD,
        timeout=20,
        allow_agent=False,
        look_for_keys=False,
    )

    def run(command: str, timeout: int = 300, live_output: bool = False):
        """Run one remote command without allowing Paramiko to wait forever.

        Paramiko's ``exec_command(timeout=...)`` does not put a total deadline
        on ``recv_exit_status()``. Also, waiting for the exit status before
        draining stdout/stderr can deadlock if a verbose command fills the SSH
        channel buffers. Drain both streams while polling and enforce our own
        wall-clock deadline.
        """
        _, stdout, stderr = client.exec_command(command, timeout=timeout)
        channel = stdout.channel
        stdout_chunks = []
        stderr_chunks = []
        deadline = time.monotonic() + timeout

        while True:
            while channel.recv_ready():
                chunk = channel.recv(32768)
                if not chunk:
                    break
                stdout_chunks.append(chunk)
                if live_output:
                    print(chunk.decode(errors="replace"), end="", flush=True)

            while channel.recv_stderr_ready():
                chunk = channel.recv_stderr(32768)
                if not chunk:
                    break
                stderr_chunks.append(chunk)
                if live_output:
                    print(
                        chunk.decode(errors="replace"),
                        end="",
                        file=sys.stderr,
                        flush=True,
                    )

            if channel.exit_status_ready():
                exit_status = channel.recv_exit_status()
                # The exit-status packet can arrive just before the last data
                # packet. Give Paramiko one final chance to expose it.
                while channel.recv_ready():
                    chunk = channel.recv(32768)
                    if not chunk:
                        break
                    stdout_chunks.append(chunk)
                    if live_output:
                        print(chunk.decode(errors="replace"), end="", flush=True)
                while channel.recv_stderr_ready():
                    chunk = channel.recv_stderr(32768)
                    if not chunk:
                        break
                    stderr_chunks.append(chunk)
                    if live_output:
                        print(
                            chunk.decode(errors="replace"),
                            end="",
                            file=sys.stderr,
                            flush=True,
                        )
                break

            if time.monotonic() >= deadline:
                channel.close()
                message = (
                    f"Remote command exceeded the {timeout}s deployment timeout."
                )
                stderr_chunks.append(("\n" + message + "\n").encode())
                return (
                    124,
                    b"".join(stdout_chunks).decode(errors="replace"),
                    b"".join(stderr_chunks).decode(errors="replace"),
                )

            time.sleep(0.05)

        return (
            exit_status,
            b"".join(stdout_chunks).decode(errors="replace"),
            b"".join(stderr_chunks).decode(errors="replace"),
        )

    sudo = f"printf '%s\\n' {shlex.quote(PASSWORD)} | sudo -S"

    sftp = client.open_sftp()
    remote_home = sftp.normalize(".")
    remote_ws = f"{remote_home}/livox_ws"
    remote_pkg = f"{remote_ws}/src/obstacle_detector"
    remote_planner_pkg = f"{remote_ws}/src/medical_clearance_planner"

    def ensure_remote_dir(path: str) -> None:
        current = "/"
        for part in path.strip("/").split("/"):
            current = posixpath.join(current, part)
            try:
                mode = sftp.stat(current).st_mode
                if not stat.S_ISDIR(mode):
                    raise RuntimeError(f"remote path is not a directory: {current}")
            except FileNotFoundError:
                sftp.mkdir(current)

    uploads = list(iter_package_files(remote_pkg, remote_planner_pkg))
    uploads.extend(
        [
            (os.path.join(VIZ, "medical_nav.rviz"), f"{remote_home}/medical_nav.rviz"),
            (os.path.join(HERE, "start_obstacle.sh"), f"{remote_home}/start_obstacle.sh"),
            (os.path.join(HERE, "start_mid360.sh"), f"{remote_home}/start_mid360.sh"),
            (os.path.join(HERE, "start_rviz.sh"), f"{remote_home}/start_rviz.sh"),
            (os.path.join(HERE, "start_viz.sh"), f"{remote_home}/start_viz.sh"),
            (os.path.join(HERE, "start_scan_dashboard.sh"),
             f"{remote_home}/start_scan_dashboard.sh"),
            (os.path.join(HERE, "monitor_velocity_layers.sh"),
             f"{remote_home}/monitor_velocity_layers.sh"),
            (os.path.join(HERE, "medical-scan-dashboard.desktop"),
             f"{remote_home}/medical-scan-dashboard.desktop"),
            (os.path.join(HERE, "mid360.service"), f"{remote_home}/mid360.service"),
            (os.path.join(HERE, "obstacle-detector.service"),
             f"{remote_home}/obstacle-detector.service"),
            (os.path.join(ROOT, "..", "check_nuc.sh"),
             f"{remote_home}/check_nuc.sh"),
        ]
    )

    for local, remote in uploads:
        if not os.path.isfile(local):
            print(f"missing local file: {local}")
            sftp.close()
            client.close()
            return 1
        ensure_remote_dir(posixpath.dirname(remote))
        sftp.put(local, remote)
        print(f"pushed {os.path.relpath(local, ROOT)} -> {remote}")
    sftp.close()

    run(
        f"chmod +x {remote_home}/start_*.sh "
        f"{remote_home}/monitor_velocity_layers.sh {remote_home}/check_nuc.sh"
    )

    print("Installing scanner, BLE health and NUC TTS runtime dependencies...")
    rc, out, err = run(
        f"{sudo} env DEBIAN_FRONTEND=noninteractive apt-get install -y "
        "python3-opencv python3-numpy python3-pyqt5 python3-pyzbar "
        "libzbar0 v4l-utils python3-pip ffmpeg alsa-utils bluez rfkill",
        timeout=600,
    )
    print(out.strip() or err.strip())
    if rc != 0:
        client.close()
        return rc

    rc, out, err = run(
        "/usr/bin/python3 -c 'import bleak' 2>/dev/null || "
        f"({sudo} env DEBIAN_FRONTEND=noninteractive apt-get install -y "
        "python3-bleak) || "
        "/usr/bin/python3 -m pip install --user bleak==0.20.2",
        timeout=300,
    )
    print(out.strip() or err.strip())
    if rc != 0:
        client.close()
        return rc

    rc, out, err = run(
        f"{sudo} systemctl enable --now bluetooth.service && "
        f"{sudo} rfkill unblock bluetooth && "
        "timeout 10s bluetoothctl power on"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc

    rc, out, err = run(
        "/usr/bin/python3 -c 'import zxingcpp' 2>/dev/null || "
        "/usr/bin/python3 -m pip install --user --no-deps zxing-cpp==3.1.1",
        timeout=300,
    )
    print(out.strip() or err.strip())
    if rc != 0:
        client.close()
        return rc

    rc, out, err = run(
        f"{sudo} usermod -aG video,audio,dialout {shlex.quote(USER)}"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc
    dashboard_desktop = f"{remote_home}/medical-scan-dashboard.desktop"
    rc, out, err = run(
        f"desktop_dir=$(xdg-user-dir DESKTOP 2>/dev/null || true); "
        f"[ -n \"$desktop_dir\" ] || desktop_dir={shlex.quote(remote_home + '/Desktop')}; "
        f"mkdir -p \"$desktop_dir\" {shlex.quote(remote_home + '/.config/autostart')} "
        f"{shlex.quote(remote_home + '/.local/share/applications')} && "
        f"install -m 0755 {shlex.quote(dashboard_desktop)} "
        f"\"$desktop_dir/medical-scan-dashboard.desktop\" && "
        f"install -m 0644 {shlex.quote(dashboard_desktop)} "
        f"{shlex.quote(remote_home + '/.config/autostart/medical-scan-dashboard.desktop')} && "
        f"install -m 0644 {shlex.quote(dashboard_desktop)} "
        f"{shlex.quote(remote_home + '/.local/share/applications/medical-scan-dashboard.desktop')}"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc
    run(
        f"desktop_dir=$(xdg-user-dir DESKTOP 2>/dev/null || true); "
        f"[ -n \"$desktop_dir\" ] || desktop_dir={shlex.quote(remote_home + '/Desktop')}; "
        "user_bus=/run/user/$(id -u)/bus; "
        "if command -v gio >/dev/null 2>&1; then "
        "if [ -S \"$user_bus\" ]; then "
        "DBUS_SESSION_BUS_ADDRESS=unix:path=$user_bus DISPLAY=${DISPLAY:-:0} "
        "gio set \"$desktop_dir/medical-scan-dashboard.desktop\" "
        "metadata::trusted true 2>/dev/null || true; "
        "else gio set \"$desktop_dir/medical-scan-dashboard.desktop\" "
        "metadata::trusted true 2>/dev/null || true; fi; fi"
    )
    # RViz is a diagnostics tool and must not consume resources on every
    # desktop login. Remove the legacy autostart entry; start_rviz.sh remains
    # available for explicit remote or local launch.
    rc, out, err = run(
        f"rm -f {remote_home}/.config/autostart/navigation-viz.desktop"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc
    rc, out, err = run(
        f"cd {remote_pkg} && /usr/bin/python3 -m py_compile "
        "obstacle_detector/nav_protocol.py obstacle_detector/serial_transport.py "
        "obstacle_detector/nuc_tts.py "
        "obstacle_detector/bridge_safety.py "
        "obstacle_detector/field_goals.py obstacle_detector/stm32_bridge.py "
        "obstacle_detector/medical_navigator.py obstacle_detector/lidar_transform.py "
        "obstacle_detector/nurse_scan_core.py "
        "obstacle_detector/lidar_odometry_core.py "
        "obstacle_detector/lidar_odometry_guard.py "
        "obstacle_detector/stp23l_calibration.py "
        "obstacle_detector/scanner_core.py obstacle_detector/code_scanner.py "
        "obstacle_detector/health_ble_core.py obstacle_detector/health_ble_bridge.py "
        "obstacle_detector/scan_dashboard_core.py obstacle_detector/scan_dashboard.py "
        "launch/obstacle.launch.py "
        "scripts/make_static_map.py"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc

    rc, out, err = run(
        # The NUC login shell may auto-activate Miniconda.  If CMake sees that
        # interpreter first, ament fails because the Conda environment does
        # not contain ROS' catkin_pkg.  Build ROS packages with Ubuntu's
        # system Python regardless of the interactive shell state.
        "unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER "
        "PYTHONHOME PYTHONPATH && "
        "export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin && "
        "source /opt/ros/humble/setup.bash && "
        f"cd {remote_ws} && colcon build --symlink-install "
        "--packages-select medical_clearance_planner obstacle_detector "
        "--cmake-clean-cache "
        "--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3"
    )
    # colcon writes its short summary to stdout and compiler diagnostics to
    # stderr.  Always print both; using `out or err` hides the actual C++
    # failure whenever the summary is non-empty.
    if out.strip():
        print(out.strip())
    if err.strip():
        print(err.strip(), file=sys.stderr)
    if rc != 0:
        client.close()
        return rc

    # Configure unattended automatic navigation with real chassis output.
    # This value persists across service restarts and NUC reboots.
    environment = (
        "MEDICAL_NAV_DRY_RUN=false\n"
        f"ROS_DOMAIN_ID={ROS_DOMAIN_ID}\n"
        "ROS_LOCALHOST_ONLY=1\n"
        f"MEDICAL_SCANNER_ENABLED={SCANNER_ENABLED}\n"
        f"MEDICAL_SCAN_CAMERA={SCAN_CAMERA}\n"
        f"MEDICAL_TELE_SCAN_CAMERA={TELE_SCAN_CAMERA}\n"
        "MEDICAL_HEALTH_BLE_ENABLED=true\n"
        f"MEDICAL_HEALTH_BLE_DEVICE_NAME={HEALTH_BLE_DEVICE_NAME}\n"
        f"MEDICAL_HEALTH_BLE_DEVICE_ADDRESS={HEALTH_BLE_DEVICE_ADDRESS}\n"
    )
    rc, out, err = run(
        f"mkdir -p {shlex.quote(remote_home + '/.config')} && "
        f"printf %s {shlex.quote(environment)} > "
        f"{shlex.quote(remote_home + '/.config/medical-navigation.env')}"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc

    systemd_steps = (
        (
            "Installing mid360.service",
            f"{sudo} install -m 0644 {remote_home}/mid360.service "
            "/etc/systemd/system/mid360.service",
            20,
        ),
        (
            "Installing obstacle-detector.service",
            f"{sudo} install -m 0644 {remote_home}/obstacle-detector.service "
            "/etc/systemd/system/obstacle-detector.service",
            20,
        ),
        ("Reloading systemd units", f"{sudo} systemctl daemon-reload", 20),
        (
            "Enabling navigation services",
            f"{sudo} systemctl enable mid360.service obstacle-detector.service",
            20,
        ),
        (
            "Restarting navigation services",
            f"{sudo} timeout --signal=TERM --kill-after=5s 45s "
            "systemctl restart mid360.service obstacle-detector.service",
            55,
        ),
    )
    rc = 0
    for step_name, command, step_timeout in systemd_steps:
        print(f"[deploy] {step_name}...", flush=True)
        rc, out, err = run(command, timeout=step_timeout, live_output=True)
        if rc != 0:
            print(
                f"[deploy] {step_name} failed with exit code {rc}.",
                file=sys.stderr,
            )
            if err.strip():
                print(err.strip(), file=sys.stderr)
            elif out.strip():
                print(out.strip(), file=sys.stderr)
            break

    if rc != 0:
        print("[deploy] Collecting systemd diagnostics...", flush=True)
        status_rc, status_out, status_err = run(
            "systemctl --no-pager -l status mid360.service "
            "obstacle-detector.service; "
            "journalctl --no-pager -n 100 -u mid360.service "
            "-u obstacle-detector.service",
            timeout=30,
        )
        del status_rc
        print(status_out.strip() or status_err.strip())

    if rc == 0:
        rc, out, err = run(
            "sleep 12 && "
            f"{sudo} systemctl is-active --quiet obstacle-detector.service && "
            "source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(remote_ws + '/install/setup.bash')} && "
            f"ROS_DOMAIN_ID={ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=1 "
            "ros2 node list 2>/dev/null | grep -Fxq /stm32_bridge && "
            f"ROS_DOMAIN_ID={ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=1 "
            "timeout 8s ros2 topic echo --once /medical_nav/bridge_status "
            "2>/dev/null | grep -Fq '\"serial\": true' && "
            f"ROS_DOMAIN_ID={ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=1 "
            "ros2 node list 2>/dev/null | grep -Fxq /health_ble_bridge && "
            f"ROS_DOMAIN_ID={ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=1 "
            "timeout 8s ros2 topic echo --once /medical_nav/health_status "
            ">/dev/null 2>&1",
            timeout=30,
        )
        if rc != 0:
            print("Navigation service started, but a required bridge is not ready.")
            status_rc, status_out, status_err = run(
                f"{sudo} systemctl status obstacle-detector.service "
                "--no-pager -l; "
                f"{sudo} journalctl -u obstacle-detector.service -n 80 --no-pager",
                timeout=30,
            )
            del status_rc
            print(status_out.strip() or status_err.strip())
    if rc == 0:
        dashboard_log = f"{remote_home}/.cache/medical-scan-dashboard.log"
        dashboard_pid = f"{remote_home}/.cache/medical-scan-dashboard.pid"
        dashboard_rc, dashboard_out, dashboard_err = run(
            f"pid_file={shlex.quote(dashboard_pid)}; "
            "if [ -r \"$pid_file\" ]; then "
            "old_pid=$(cat \"$pid_file\"); "
            "case \"$old_pid\" in (*[!0-9]*|'') ;; "
            "(*) if [ -r \"/proc/$old_pid/cmdline\" ] && "
            "tr '\\0' ' ' < \"/proc/$old_pid/cmdline\" | "
            "grep -Eq '(/scan_dashboard|ros2 run obstacle_detector scan_dashboard)'; "
            "then kill \"$old_pid\" 2>/dev/null || true; fi ;; esac; fi; "
            "pkill -f '[/]scan_dashboard$' 2>/dev/null || true; "
            "sleep 1; "
            f"mkdir -p {shlex.quote(remote_home + '/.cache')}; "
            "user_bus=/run/user/$(id -u)/bus; "
            f"nohup env DISPLAY=:0 DBUS_SESSION_BUS_ADDRESS=unix:path=$user_bus "
            f"{shlex.quote(remote_home + '/start_scan_dashboard.sh')} "
            f"</dev/null >{shlex.quote(dashboard_log)} 2>&1 & "
            "sleep 2; "
            "new_pid=$(cat \"$pid_file\" 2>/dev/null || true); "
            "case \"$new_pid\" in (*[!0-9]*|'') exit 1 ;; "
            "(*) kill -0 \"$new_pid\" 2>/dev/null ;; esac",
            timeout=20,
        )
        if dashboard_rc != 0:
            print("Scan dashboard failed to restart after deployment.")
            print(dashboard_err or dashboard_out)
            rc = dashboard_rc
    client.close()
    if rc == 0:
        print("Deployment complete in AUTORUN mode (DRY-RUN=false).")
        print(
            f"Scanner: enabled={SCANNER_ENABLED} camera={SCAN_CAMERA} "
            f"tele_camera={TELE_SCAN_CAMERA}"
        )
        print("Scan dashboard: desktop launcher installed and login autostart enabled.")
        print("WARNING: powering or rebooting the robot may start chassis motion automatically.")
        print("RViz autostart is disabled; run ~/start_rviz.sh when needed.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
