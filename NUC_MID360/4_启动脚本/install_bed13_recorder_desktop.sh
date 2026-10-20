#!/usr/bin/env bash
set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
recorder_source="$script_dir/record_bed13_local.sh"
desktop_source="$script_dir/bed13-rosbag.desktop"

for source_file in "$recorder_source" "$desktop_source"; do
    if [ ! -r "$source_file" ]; then
        printf 'Missing Bed13 recorder file: %s\n' "$source_file" >&2
        exit 1
    fi
done

desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [ -z "$desktop_dir" ] || [ "$desktop_dir" = "$HOME" ]; then
    desktop_dir="$HOME/Desktop"
fi
applications_dir="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$desktop_dir" "$applications_dir"

install -m 0755 "$recorder_source" "$HOME/record_bed13_local.sh"
install -m 0755 "$desktop_source" "$desktop_dir/bed13-rosbag.desktop"
install -m 0644 "$desktop_source" "$applications_dir/bed13-rosbag.desktop"

if command -v gio >/dev/null 2>&1; then
    session_bus="/run/user/$(id -u)/bus"
    if [ -S "$session_bus" ]; then
        export DBUS_SESSION_BUS_ADDRESS="unix:path=$session_bus"
    fi
    if ! timeout 5s gio set "$desktop_dir/bed13-rosbag.desktop" \
        metadata::trusted true 2>/dev/null; then
        printf '%s\n' \
            'If launching is blocked, right-click the icon and select Allow Launching.'
    fi
fi

printf 'Bed13 recorder shortcut installed: %s\n' \
    "$desktop_dir/bed13-rosbag.desktop"
