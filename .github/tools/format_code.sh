#!/usr/bin/env bash

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"

# Source ROS 2 environment

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
fi

# Packages to format

PACKAGE_PATHS=(
  "tiago_autonomy_bringup"
  "tiago_navigation"
)

# Help

usage() {
  cat <<'EOF'
Usage:
  ./.github/tools/format_code.sh          Format source files in place
  ./.github/tools/format_code.sh --check  Check formatting without modifying files
  ./.github/tools/format_code.sh --help   Show this message

Formatters:
  Python : Black
  C/C++  : ROS 2 ament_clang_format
EOF
}

# Dependency check

require_command() {
  local command_name="$1"
  local install_hint="$2"

  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: '${command_name}' is not installed."
    echo
    echo "Install it with:"
    echo "  ${install_hint}"
    exit 1
  fi
}

# Arguments

MODE="format"

case "${1:-}" in
  "")
    MODE="format"
    ;;

  --check)
    MODE="check"
    ;;

  --help|-h)
    usage
    exit 0
    ;;

  *)
    echo "ERROR: Unknown argument: $1"
    echo
    usage
    exit 2
    ;;
esac

# Verify tools

require_command \
  black \
  "python3 -m pip install black"

require_command \
  ament_clang_format \
  "sudo apt install ros-${ROS_DISTRO_NAME}-ament-clang-format"

# Verify package paths

for package_path in "${PACKAGE_PATHS[@]}"; do
  if [[ ! -d "${package_path}" ]]; then
    echo "ERROR: Package directory does not exist:"
    echo "  ${REPO_ROOT}/${package_path}"
    exit 1
  fi
done

# Information

echo "Repository : ${REPO_ROOT}"
echo "ROS distro : ${ROS_DISTRO_NAME}"
echo "Mode       : ${MODE}"

# Formatting

if [[ "${MODE}" == "check" ]]; then
  echo
  echo "==> Checking Python formatting with Black"

  black --check "${PACKAGE_PATHS[@]}"

  echo
  echo "==> Checking C/C++ formatting with ament_clang_format"

  ament_clang_format "${PACKAGE_PATHS[@]}"

  echo
  echo "Formatting check passed."
else
  echo
  echo "==> Formatting Python with Black"

  black "${PACKAGE_PATHS[@]}"

  echo
  echo "==> Formatting C/C++ with ament_clang_format"

  ament_clang_format --reformat "${PACKAGE_PATHS[@]}"

  echo
  echo "Formatting complete."
  echo
  echo "Review the changes with:"
  echo "  git diff"
fi