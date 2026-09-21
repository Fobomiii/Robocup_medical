#!/bin/bash
set -e

case "${1:-rviz}" in
    lidar)
        exec "$HOME/start_lidar_viz.sh"
        ;;
    pyqt)
        exec "$HOME/start_pyqt_viz.sh"
        ;;
    *)
        exec "$HOME/start_rviz.sh"
        ;;
esac
