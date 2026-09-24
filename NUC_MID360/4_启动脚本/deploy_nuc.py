#!/usr/bin/env python3
"""Deploy navigation, scanning and the fullscreen NUC dashboard."""

import os
import posixpath
import shlex
import stat
import sys

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

    def run(command: str, timeout: int = 300):
        _, stdout, stderr = client.exec_command(command, timeout=timeout)
        return (
            stdout.channel.recv_exit_status(),
            stdout.read().decode(errors="replace"),
            stderr.read().decode(errors="replace"),
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

    run(f"chmod +x {remote_home}/start_*.sh {remote_home}/check_nuc.sh")

    print("Installing scanner runtime dependencies...")
    rc, out, err = run(
        f"{sudo} env DEBIAN_FRONTEND=noninteractive apt-get install -y "
        "python3-opencv python3-numpy python3-pyqt5 python3-pyzbar "
        "libzbar0 v4l-utils python3-pip",
        timeout=600,
    )
    print(out.strip() or err.strip())
    if rc != 0:
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

    rc, out, err = run(f"{sudo} usermod -aG video {shlex.quote(USER)}")
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
        "obstacle_detector/bridge_safety.py "
        "obstacle_detector/field_goals.py obstacle_detector/stm32_bridge.py "
        "obstacle_detector/medical_navigator.py obstacle_detector/lidar_transform.py "
        "obstacle_detector/nurse_scan_core.py "
        "obstacle_detector/lidar_odometry_core.py "
        "obstacle_detector/lidar_odometry_guard.py "
        "obstacle_detector/stp23l_calibration.py "
        "obstacle_detector/scanner_core.py obstacle_detector/code_scanner.py "
        "obstacle_detector/scan_dashboard_core.py obstacle_detector/scan_dashboard.py "
        "launch/obstacle.launch.py "
        "scripts/make_static_map.py"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc

    rc, out, err = run(
        "source /opt/ros/humble/setup.bash && "
        f"cd {remote_ws} && colcon build --symlink-install "
        "--packages-select medical_clearance_planner obstacle_detector"
    )
    print(out.strip() or err.strip())
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

    rc, out, err = run(
        f"{sudo} install -m 0644 {remote_home}/mid360.service "
        "/etc/systemd/system/mid360.service && "
        f"{sudo} install -m 0644 {remote_home}/obstacle-detector.service "
        "/etc/systemd/system/obstacle-detector.service && "
        f"{sudo} systemctl daemon-reload && "
        f"{sudo} systemctl enable mid360.service obstacle-detector.service && "
        f"{sudo} systemctl restart mid360.service obstacle-detector.service"
    )
    print(out.strip() or err.strip())
    if rc == 0:
        dashboard_log = f"{remote_home}/.cache/medical-scan-dashboard.log"
        dashboard_rc, dashboard_out, dashboard_err = run(
            "pkill -f '[s]can_dashboard' 2>/dev/null || true; "
            "sleep 1; "
            f"mkdir -p {shlex.quote(remote_home + '/.cache')}; "
            "user_bus=/run/user/$(id -u)/bus; "
            f"nohup env DISPLAY=:0 DBUS_SESSION_BUS_ADDRESS=unix:path=$user_bus "
            f"{shlex.quote(remote_home + '/start_scan_dashboard.sh')} "
            f"</dev/null >{shlex.quote(dashboard_log)} 2>&1 &",
            timeout=20,
        )
        if dashboard_rc != 0:
            print(dashboard_err or dashboard_out)
    client.close()
    if rc == 0:
        print("Deployment complete in AUTORUN mode (DRY-RUN=false).")
        print(f"Scanner: enabled={SCANNER_ENABLED} camera={SCAN_CAMERA}")
        print("Scan dashboard: desktop launcher installed and login autostart enabled.")
        print("WARNING: powering or rebooting the robot may start chassis motion automatically.")
        print("RViz autostart is disabled; run ~/start_rviz.sh when needed.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
