#!/usr/bin/env python3
"""Fit a temporary RViz overview to the NUC's available desktop geometry."""

import argparse
from copy import deepcopy
from pathlib import Path

import yaml


def fit_config(config, width, height, left=0, top=0):
    if width <= 0 or height <= 0:
        raise ValueError("Screen width and height must be positive")
    fitted = deepcopy(config)
    window = fitted.setdefault("Window Geometry", {})
    window.pop("QMainWindow State", None)
    window.update({
        "Width": int(width * 0.96), "Height": int(height * 0.94),
        "X": left + int(width * 0.02), "Y": top + int(height * 0.02),
        "Hide Left Dock": width < 1100, "Hide Right Dock": True,
    })
    dock_width = 0 if window["Hide Left Dock"] else 360
    viewport_width = max(100, window["Width"] - dock_width - 40)
    viewport_height = max(100, window["Height"] - 120)
    fitted["Visualization Manager"]["Views"]["Current"] = {
        "Class": "rviz_default_plugins/TopDownOrtho",
        "Name": "Field Overview",
        "Angle": 0.0,
        "Scale": round(min(viewport_width, viewport_height) / 8.0, 3),
        "X": 3.0,
        "Y": 0.0,
        "Target Frame": "map",
        "Near Clip Distance": 0.01,
    }
    for panel in fitted.get("Panels", []):
        if panel.get("Class") == "rviz_common/Displays":
            panel["Tree Height"] = max(100, window["Height"] - 180)
    return fitted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    if arguments.source.resolve() == arguments.output.resolve():
        parser.error("The runtime output must not overwrite the source RViz configuration")

    from PyQt5.QtWidgets import QApplication

    application = QApplication(["medical-rviz-screen"])
    screen = application.primaryScreen()
    if screen is None:
        raise RuntimeError("No desktop screen is available for RViz")
    geometry = screen.availableGeometry()
    config = yaml.safe_load(arguments.source.read_text(encoding="utf-8"))
    fitted = fit_config(
        config, geometry.width(), geometry.height(), geometry.x(), geometry.y()
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        yaml.safe_dump(fitted, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    view = fitted["Visualization Manager"]["Views"]["Current"]
    print(
        f"RViz desktop={geometry.width()}x{geometry.height()}, "
        f"overview scale={view['Scale']} px/m, config={arguments.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
