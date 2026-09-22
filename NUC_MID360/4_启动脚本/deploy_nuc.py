#!/usr/bin/env python3
"""Deploy the complete Nav2 package, services and login-time RViz to the NUC."""

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

REMOTE_HOME = f"/home/{USER}"
REMOTE_WS = f"{REMOTE_HOME}/livox_ws"
REMOTE_PKG = f"{REMOTE_WS}/src/obstacle_detector"
REMOTE_PLANNER_PKG = f"{REMOTE_WS}/src/medical_clearance_planner"


def iter_package_files():
    packages = (
        (PKG, REMOTE_PKG),
        (PLANNER_PKG, REMOTE_PLANNER_PKG),
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

    uploads = list(iter_package_files())
    uploads.extend(
        [
            (os.path.join(VIZ, "medical_nav.rviz"), f"{REMOTE_HOME}/medical_nav.rviz"),
            (os.path.join(HERE, "start_obstacle.sh"), f"{REMOTE_HOME}/start_obstacle.sh"),
            (os.path.join(HERE, "start_mid360.sh"), f"{REMOTE_HOME}/start_mid360.sh"),
            (os.path.join(HERE, "start_rviz.sh"), f"{REMOTE_HOME}/start_rviz.sh"),
            (os.path.join(HERE, "start_viz.sh"), f"{REMOTE_HOME}/start_viz.sh"),
            (os.path.join(HERE, "mid360.service"), f"{REMOTE_HOME}/mid360.service"),
            (os.path.join(HERE, "obstacle-detector.service"),
             f"{REMOTE_HOME}/obstacle-detector.service"),
            (os.path.join(ROOT, "..", "check_nuc.sh"),
             f"{REMOTE_HOME}/check_nuc.sh"),
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

    run(f"chmod +x {REMOTE_HOME}/start_*.sh {REMOTE_HOME}/check_nuc.sh")

    print("Installing scanner runtime dependencies...")
    rc, out, err = run(
        f"{sudo} env DEBIAN_FRONTEND=noninteractive apt-get install -y "
        "python3-opencv python3-numpy python3-pyzbar libzbar0 v4l-utils python3-pip",
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
    # RViz is a diagnostics tool and must not consume resources on every
    # desktop login. Remove the legacy autostart entry; start_rviz.sh remains
    # available for explicit remote or local launch.
    rc, out, err = run(
        f"rm -f {REMOTE_HOME}/.config/autostart/navigation-viz.desktop"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc
    rc, out, err = run(
        f"cd {REMOTE_PKG} && /usr/bin/python3 -m py_compile "
        "obstacle_detector/nav_protocol.py obstacle_detector/serial_transport.py "
        "obstacle_detector/field_goals.py obstacle_detector/stm32_bridge.py "
        "obstacle_detector/medical_navigator.py obstacle_detector/lidar_transform.py "
        "obstacle_detector/lidar_odometry_core.py "
        "obstacle_detector/lidar_odometry_guard.py "
        "obstacle_detector/stp23l_calibration.py "
        "obstacle_detector/scanner_core.py obstacle_detector/code_scanner.py "
        "launch/obstacle.launch.py "
        "scripts/make_static_map.py"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc

    rc, out, err = run(
        "source /opt/ros/humble/setup.bash && "
        f"cd {REMOTE_WS} && colcon build --symlink-install "
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
        f"mkdir -p {shlex.quote(REMOTE_HOME + '/.config')} && "
        f"printf %s {shlex.quote(environment)} > "
        f"{shlex.quote(REMOTE_HOME + '/.config/medical-navigation.env')}"
    )
    if rc != 0:
        print(err or out)
        client.close()
        return rc

    rc, out, err = run(
        f"{sudo} install -m 0644 {REMOTE_HOME}/mid360.service "
        "/etc/systemd/system/mid360.service && "
        f"{sudo} install -m 0644 {REMOTE_HOME}/obstacle-detector.service "
        "/etc/systemd/system/obstacle-detector.service && "
        f"{sudo} systemctl daemon-reload && "
        f"{sudo} systemctl enable mid360.service obstacle-detector.service && "
        f"{sudo} systemctl restart mid360.service obstacle-detector.service"
    )
    print(out.strip() or err.strip())
    client.close()
    if rc == 0:
        print("Deployment complete in AUTORUN mode (DRY-RUN=false).")
        print(f"Scanner: enabled={SCANNER_ENABLED} camera={SCAN_CAMERA}")
        print("WARNING: powering or rebooting the robot may start chassis motion automatically.")
        print("RViz autostart is disabled; run ~/start_rviz.sh when needed.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
