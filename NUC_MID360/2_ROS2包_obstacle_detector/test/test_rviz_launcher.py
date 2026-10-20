"""Desktop-only RViz sizing and installation regressions."""

import configparser
from copy import deepcopy
import importlib.util
from pathlib import Path
import shlex
import unittest

import yaml


NUC_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = NUC_ROOT / "4_启动脚本"


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RvizLauncherTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sizing = load_script("prepare_rviz_config")
        cls.config = yaml.safe_load(
            (NUC_ROOT / "3_可视化工具" / "medical_nav.rviz").read_text(
                encoding="utf-8"
            )
        )

    def test_overview_fits_common_displays_without_mutating_source(self):
        original = deepcopy(self.config)
        for width, height in ((800, 480), (1024, 600), (1920, 1080)):
            fitted = self.sizing.fit_config(self.config, width, height)
            window = fitted["Window Geometry"]
            self.assertLessEqual(window["X"] + window["Width"], width)
            self.assertLessEqual(window["Y"] + window["Height"], height)
            view = fitted["Visualization Manager"]["Views"]["Current"]
            self.assertEqual(view["Class"], "rviz_default_plugins/TopDownOrtho")
            self.assertEqual(view["Target Frame"], "map")
        self.assertEqual(self.config, original)

    def test_shortcut_is_visible_and_uses_explicit_launcher(self):
        desktop = configparser.ConfigParser(interpolation=None)
        desktop.read(SCRIPTS / "navigation-viz.desktop", encoding="utf-8")
        entry = desktop["Desktop Entry"]
        self.assertEqual(entry["Type"], "Application")
        self.assertEqual(entry["Terminal"], "false")
        self.assertNotEqual(entry.get("Hidden"), "true")
        self.assertEqual(
            shlex.split(entry["Exec"]), ["/bin/bash", "-c", "exec ~/start_rviz.sh"]
        )

    def test_installer_is_manual_only_and_deploys_all_files(self):
        launcher = (SCRIPTS / "start_rviz.sh").read_text(encoding="utf-8")
        installer = (SCRIPTS / "install_rviz_desktop.sh").read_text(
            encoding="utf-8"
        )
        deploy = (SCRIPTS / "deploy_nuc.py").read_text(encoding="utf-8")
        self.assertIn('exec rviz2 -d "$RUNTIME_CONFIG"', launcher)
        self.assertIn("metadata::trusted true", installer)
        self.assertNotIn(".config/autostart", installer)
        for forbidden in ("systemctl", "pkill", "obstacle.launch.py"):
            self.assertNotIn(forbidden, launcher)
            self.assertNotIn(forbidden, installer)
        for name in (
            "prepare_rviz_config.py",
            "install_rviz_desktop.sh",
            "navigation-viz.desktop",
        ):
            self.assertIn(f'os.path.join(HERE, "{name}")', deploy)
            self.assertIn(f'f"{{remote_home}}/{name}"', deploy)


if __name__ == "__main__":
    unittest.main()
