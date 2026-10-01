"""Exercise the installed assets that a fresh clone needs to launch."""

from pathlib import Path
import runpy

from ament_index_python.packages import get_package_prefix


def test_installed_assignment_assets():
    script = Path(get_package_prefix('ur_task_planner')) / 'lib/ur_task_planner/check_install.py'
    runpy.run_path(str(script))['check_installation']()
