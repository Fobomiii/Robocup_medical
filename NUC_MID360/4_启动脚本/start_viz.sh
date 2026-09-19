#!/bin/bash
set -e

case "${1:-rviz}" in
    lidar)
        exec /home/gp-pcie/start_lidar_viz.sh
        ;;
    pyqt)
        exec /home/gp-pcie/start_pyqt_viz.sh
        ;;
    *)
        exec /home/gp-pcie/start_rviz.sh
        ;;
esac
