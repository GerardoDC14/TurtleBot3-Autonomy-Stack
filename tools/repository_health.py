#!/usr/bin/env python3
"""Fast, dependency-free repository checks for the TurtleBot3 workspace."""

from __future__ import annotations

import ast
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[1]

FIRST_PARTY_PACKAGES = (
    "autonomy_common",
    "explore_cpp",
    "turtlebot3_explore",
    "turtlebot3_poi_navigation",
    "laser_scan_adjuster",
    "hazmat_marker",
    "motor_position_controller",
    "joint_state_publisher_custom",
)

REQUIRED_PATHS = (
    "README.md",
    ".gitignore",
    ".gitmodules",
    "docs/ARCHITECTURE.md",
    "docs/OPERATIONS.md",
    "docs/PROJECT_STATUS.md",
)

PLACEHOLDERS = (
    "todo: package description",
    "todo: license declaration",
    "your name",
    "tu nombre",
    "your_email@example.com",
    "your.email@example.com",
    "tu_email@example.com",
    "gerardo@todo.todo",
)

FORBIDDEN_DIRECTORY_NAMES = {"build", "install", "log", "__pycache__"}


def error(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr)


def check_required_paths() -> list[str]:
    failures: list[str] = []
    for relative in REQUIRED_PATHS:
        if not (ROOT / relative).exists():
            failures.append(f"missing required path: {relative}")
    return failures


def check_generated_directories() -> list[str]:
    failures: list[str] = []
    result = subprocess.run(
        ["git", "ls-files", "-z"], cwd=ROOT, capture_output=True, check=True
    )
    for name in result.stdout.decode("utf-8").split("\0"):
        if name and FORBIDDEN_DIRECTORY_NAMES.intersection(Path(name).parts):
            failures.append(f"generated artifact is tracked: {name}")
    return failures


def check_package_metadata() -> list[str]:
    failures: list[str] = []
    for package_name in FIRST_PARTY_PACKAGES:
        package_dir = ROOT / "src" / package_name
        package_xml = package_dir / "package.xml"
        if not package_xml.exists():
            failures.append(f"{package_name}: package.xml is missing")
            continue

        try:
            root = ElementTree.parse(package_xml).getroot()
        except ElementTree.ParseError as exc:
            failures.append(f"{package_name}: invalid package.xml: {exc}")
            continue

        xml_name = (root.findtext("name") or "").strip()
        description = (root.findtext("description") or "").strip()
        maintainer = root.find("maintainer")
        license_text = (root.findtext("license") or "").strip()

        if xml_name != package_name:
            failures.append(f"{package_name}: package name is {xml_name!r}")
        if not description:
            failures.append(f"{package_name}: description is empty")
        if maintainer is None or not (maintainer.text or "").strip():
            failures.append(f"{package_name}: maintainer is missing")
        if not license_text:
            failures.append(f"{package_name}: license field is empty")

        metadata_text = package_xml.read_text(encoding="utf-8").lower()
        for placeholder in PLACEHOLDERS:
            if placeholder in metadata_text:
                failures.append(f"{package_name}: placeholder metadata remains: {placeholder}")

        if not ((package_dir / "setup.py").exists() or (package_dir / "CMakeLists.txt").exists()):
            failures.append(f"{package_name}: no setup.py or CMakeLists.txt")

    return failures


def check_python_syntax() -> tuple[list[str], int]:
    failures: list[str] = []
    checked = 0
    for package_name in FIRST_PARTY_PACKAGES:
        package_dir = ROOT / "src" / package_name
        for path in package_dir.rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            checked += 1
            try:
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            except (SyntaxError, UnicodeDecodeError) as exc:
                failures.append(f"{path.relative_to(ROOT)}: {exc}")
    return failures, checked


def main() -> int:
    failures: list[str] = []
    failures.extend(check_required_paths())
    failures.extend(check_generated_directories())
    failures.extend(check_package_metadata())
    syntax_failures, python_count = check_python_syntax()
    failures.extend(syntax_failures)

    if failures:
        for failure in failures:
            error(failure)
        print(f"Repository health check failed with {len(failures)} issue(s).", file=sys.stderr)
        return 1

    print(
        "Repository health check passed: "
        f"{len(FIRST_PARTY_PACKAGES)} ROS packages and {python_count} Python files validated."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
