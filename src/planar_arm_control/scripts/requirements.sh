#!/usr/bin/env bash
#
# planar_arm_control -- task dependency installer
#
# Usage:
#   ./requirements.sh              install what is missing
#   ./requirements.sh --check      report what is missing, install nothing

set -euo pipefail

# Note: Adjust these if you are targeting Humble on 22.04. 
# Defaults here are set for Jazzy on Ubuntu 24.04 based on your previous script.
ROS_DISTRO_REQUIRED="jazzy"
UBUNTU_VERSION_REQUIRED="24.04"

ROS_PACKAGES_EXPECTED=(
  ros-${ROS_DISTRO_REQUIRED}-rclpy
  ros-${ROS_DISTRO_REQUIRED}-sensor-msgs
  ros-${ROS_DISTRO_REQUIRED}-geometry-msgs
  ros-${ROS_DISTRO_REQUIRED}-std-msgs
  ros-${ROS_DISTRO_REQUIRED}-launch-ros
)

PYTHON_PACKAGES=(
  python3-numpy
  python3-pyqt5      # Safer than pip install on Ubuntu 24.04 (PEP 668)
  python3-pyqtgraph  # Safer than pip install on Ubuntu 24.04 (PEP 668)
  python3-pip
)

BUILD_PACKAGES=(
  build-essential
  python3-colcon-common-extensions
  python3-rosdep
)

if [ -t 1 ]; then
  C_RESET=$'\033[0m'; C_BOLD=$'\033[1m'; C_DIM=$'\033[2m'
  C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_CYAN=$'\033[36m'
else
  C_RESET=""; C_BOLD=""; C_DIM=""; C_RED=""; C_GREEN=""; C_YELLOW=""; C_CYAN=""
fi

section() { printf '\n%s%s%s\n' "$C_BOLD" "$1" "$C_RESET"; }
ok()      { printf '  %s[ ok ]%s %s\n' "$C_GREEN" "$C_RESET" "$1"; }
missing() { printf '  %s[miss]%s %s\n' "$C_YELLOW" "$C_RESET" "$1"; }
fail()    { printf '  %s[fail]%s %s\n' "$C_RED" "$C_RESET" "$1"; }
info()    { printf '  %s%s%s\n' "$C_DIM" "$1" "$C_RESET"; }
die()     { printf '\n%serror:%s %s\n\n' "$C_RED" "$C_RESET" "$1" >&2; exit 1; }

rule() {
  printf '%s%s%s\n' "$C_CYAN" "==================================================================" "$C_RESET"
}

CHECK_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    *) die "unrecognised argument '$1'. Usage: ./requirements.sh [--check]" ;;
  esac
  shift
done

printf '\n'
rule
printf '  %splanar_arm_control  |  task dependency installer%s\n' "$C_BOLD" "$C_RESET"
rule

section "Checking the system"

[ -r /etc/os-release ] || die "cannot read /etc/os-release; this is not a supported system."
. /etc/os-release

if [ "${ID:-}" != "ubuntu" ]; then
  die "This task requires Ubuntu. Other distributions are not supported."
fi

ok "Ubuntu $VERSION_ID ($VERSION_CODENAME)"

apt_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

apt_available() {
  local candidate
  candidate=$(apt-cache policy "$1" 2>/dev/null | awk '/Candidate:/ {print $2}')
  [ -n "$candidate" ] && [ "$candidate" != "(none)" ]
}

section "ROS 2 Prerequisites"

if [ -d "/opt/ros/$ROS_DISTRO_REQUIRED" ]; then
  ok "/opt/ros/$ROS_DISTRO_REQUIRED exists"
else
  fail "/opt/ros/$ROS_DISTRO_REQUIRED does not exist -- ROS 2 $ROS_DISTRO_REQUIRED is not installed"
  die "Please install ROS 2 $ROS_DISTRO_REQUIRED first."
fi

APT_UPDATED=0
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  command -v sudo >/dev/null 2>&1 || die "sudo is not installed, and this is not running as root."
  SUDO="sudo"
fi

apt_update_once() {
  [ "$APT_UPDATED" -eq 1 ] && return 0
  info "refreshing the package lists..."
  $SUDO apt-get update -qq || die "apt-get update failed. Check your network connection."
  APT_UPDATED=1
}

if [ "$CHECK_ONLY" -eq 0 ]; then
  if [ -n "$SUDO" ]; then
    info "apt needs root; you may be asked for your password."
    $SUDO -v || die "could not obtain sudo privileges."
  fi
  apt_update_once
fi

TO_INSTALL=()
UNAVAILABLE=()

survey_group() {
  local label="$1"; shift
  section "$label"
  local pkg
  for pkg in "$@"; do
    if apt_installed "$pkg"; then
      ok "$pkg"
    elif apt_available "$pkg"; then
      missing "$pkg"
      TO_INSTALL+=("$pkg")
    else
      fail "$pkg -- not found in any configured apt repository"
      UNAVAILABLE+=("$pkg")
    fi
  done
}

survey_group "ROS 2 Client Libraries & Messages" "${ROS_PACKAGES_EXPECTED[@]}"
survey_group "Python GUI Modules" "${PYTHON_PACKAGES[@]}"
survey_group "Build Tools" "${BUILD_PACKAGES[@]}"

section "Summary"

if [ "${#UNAVAILABLE[@]}" -gt 0 ]; then
  fail "${#UNAVAILABLE[@]} package(s) could not be found in any configured repository:"
  for pkg in "${UNAVAILABLE[@]}"; do
    info "  $pkg"
  done
  die "Your apt lists may be stale. Try: sudo apt update and run this again."
fi

if [ "${#TO_INSTALL[@]}" -eq 0 ]; then
  ok "nothing to install -- every task dependency is already present"
else
  printf '  %d package(s) to install:\n' "${#TO_INSTALL[@]}"
  for pkg in "${TO_INSTALL[@]}"; do
    info "  $pkg"
  done

  if [ "$CHECK_ONLY" -eq 1 ]; then
    printf '\n  would run: %s apt-get install -y %s\n' "$SUDO" "${TO_INSTALL[*]}"
    printf '  %sRun without --check to install them.%s\n\n' "$C_DIM" "$C_RESET"
    exit 0
  fi

  section "Installing"
  if ! $SUDO apt-get install -y "${TO_INSTALL[@]}"; then
    die "apt-get install failed."
  fi
  ok "packages installed"
fi

printf '\n'
ok "All dependencies are ready for planar_arm_control!"
printf '\n'