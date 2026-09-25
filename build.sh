#!/usr/bin/env bash
# Build the workspace with the layout run_go2_teleop.sh expects
# (--merge-install --symlink-install).
#
# Usage:
#   ./build.sh                   # incremental build
#   ./build.sh --clean           # wipe build/ install/ log/ first
#   ./build.sh --deps            # install ROS + Python deps (rosdep, apt) first
#   ./build.sh --test            # run colcon test after building
#   ./build.sh --pkg go2_eskf    # build (and test) only this package; repeatable
#   ./build.sh --clean --deps --test
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"

CLEAN="false"
DEPS="false"
TEST="false"
PKGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean) CLEAN="true" ;;
    --deps)  DEPS="true" ;;
    --test)  TEST="true" ;;
    --pkg)   shift; PKGS+=("${1:?--pkg needs a package name}") ;;
    -h|--help) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 1 ;;
  esac
  shift
done

[[ -f "$ROS_DISTRO_SETUP" ]] || { echo "ERROR: $ROS_DISTRO_SETUP not found — install ROS 2 Jazzy." >&2; exit 1; }
# ROS setup scripts reference unset variables; relax -u while sourcing.
set +u; source "$ROS_DISTRO_SETUP"; set -u

cd "$WS"

if [[ "$CLEAN" == "true" ]]; then
  echo "==> Cleaning build/ install/ log/"
  rm -rf build install log
fi

if [[ "$DEPS" == "true" ]]; then
  echo "==> Installing ROS dependencies (rosdep)"
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
  echo "==> Installing Python dependencies (apt; see requirements.txt)"
  sudo apt-get install -y python3-numpy python3-matplotlib python3-pil
fi

SELECT=()
[[ ${#PKGS[@]} -gt 0 ]] && SELECT=(--packages-select "${PKGS[@]}")

echo "==> colcon build"
colcon build --merge-install --symlink-install "${SELECT[@]}"

if [[ "$TEST" == "true" ]]; then
  echo "==> colcon test"
  colcon test --merge-install "${SELECT[@]}"
  colcon test-result --verbose
fi

echo
echo "Done. Load the workspace with:  source $WS/install/setup.bash"
