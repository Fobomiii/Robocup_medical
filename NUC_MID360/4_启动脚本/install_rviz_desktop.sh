#!/bin/bash
set -e

for rviz_file in start_rviz.sh prepare_rviz_config.py medical_nav.rviz navigation-viz.desktop; do
    if [ ! -r "$HOME/$rviz_file" ]; then
        printf 'Missing RViz launcher file: %s\n' "$HOME/$rviz_file" >&2
        exit 1
    fi
done

desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [ -z "$desktop_dir" ] || [ "$desktop_dir" = "$HOME" ]; then
    desktop_dir="$HOME/Desktop"
fi
applications_dir="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$desktop_dir" "$applications_dir"
chmod +x "$HOME/start_rviz.sh"
install -m 0755 "$HOME/navigation-viz.desktop" "$desktop_dir/navigation-viz.desktop"
install -m 0644 "$HOME/navigation-viz.desktop" "$applications_dir/navigation-viz.desktop"

if command -v gio >/dev/null 2>&1; then
    session_bus="/run/user/$(id -u)/bus"
    if [ -S "$session_bus" ]; then
        export DBUS_SESSION_BUS_ADDRESS="unix:path=$session_bus"
    fi
    if ! timeout 5s gio set "$desktop_dir/navigation-viz.desktop" metadata::trusted true 2>/dev/null; then
        printf '%s\n' 'If the desktop blocks launching, right-click the RViz icon and select Allow Launching.'
    fi
fi
printf 'RViz desktop shortcut installed: %s\n' "$desktop_dir/navigation-viz.desktop"
