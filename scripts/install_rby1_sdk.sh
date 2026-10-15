#!/usr/bin/env bash
# Build the rby1-sdk C++ library from source and install it with its CMake package
# (lib/cmake/rby1-sdk), so that ros2 packages can use find_package(rby1-sdk CONFIG).
#
# The pip wheel (python3 -m pip install rby1-sdk) only ships the Python module and
# cannot be used to build the C++ hardware interface.
#
# Usage:
#   scripts/install_rby1_sdk.sh [--prefix DIR] [--version TAG] [--jobs N] [--work-dir DIR]
#
# Defaults: --prefix /opt/rby1-sdk when writable (or with sudo), otherwise
# ~/.local/rby1-sdk; --version v0.10.0. Works on amd64 and arm64 (Jetson).
# Afterwards build the workspace with CMAKE_PREFIX_PATH containing the prefix, e.g.
#   export CMAKE_PREFIX_PATH=<prefix>:$CMAKE_PREFIX_PATH
#   export LD_LIBRARY_PATH=<prefix>/lib:$LD_LIBRARY_PATH
set -euo pipefail

VERSION="v0.10.0"
PREFIX=""
JOBS="$(nproc)"
WORK_DIR=""
SDK_REPO="https://github.com/RainbowRobotics/rby1-sdk.git"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix) PREFIX="$2"; shift 2 ;;
    --version) VERSION="$2"; shift 2 ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

SUDO=""
if [[ -z "${PREFIX}" ]]; then
  if [[ -w /opt ]] || [[ "$(id -u)" -eq 0 ]]; then
    PREFIX="/opt/rby1-sdk"
  elif command -v sudo >/dev/null && sudo -n true 2>/dev/null; then
    PREFIX="/opt/rby1-sdk"
    SUDO="sudo"
  else
    PREFIX="${HOME}/.local/rby1-sdk"
  fi
fi
if [[ -e "${PREFIX}" && ! -w "${PREFIX}" ]] || [[ ! -e "${PREFIX}" && ! -w "$(dirname "${PREFIX}")" ]]; then
  SUDO="sudo"
fi

for tool in git cmake python3 make g++; do
  command -v "${tool}" >/dev/null || { echo "missing required tool: ${tool}" >&2; exit 1; }
done

# Conan 2 in an isolated venv so the system Python stays untouched.
CONAN_VENV="${RBY1_SDK_CONAN_VENV:-${HOME}/.cache/rby1-sdk/conan-venv}"
if [[ ! -x "${CONAN_VENV}/bin/conan" ]]; then
  python3 -m venv "${CONAN_VENV}"
  "${CONAN_VENV}/bin/pip" install --quiet --upgrade pip "conan>=2.0,<3"
fi
CONAN="${CONAN_VENV}/bin/conan"
"${CONAN}" profile detect --exist-ok >/dev/null

if [[ -z "${WORK_DIR}" ]]; then
  WORK_DIR="$(mktemp -d -t rby1-sdk-build-XXXXXX)"
  trap 'rm -rf "${WORK_DIR}"' EXIT
fi
SRC="${WORK_DIR}/rby1-sdk"
if [[ ! -d "${SRC}/.git" ]]; then
  git clone --depth 1 --branch "${VERSION}" --recurse-submodules --shallow-submodules "${SDK_REPO}" "${SRC}"
fi

cd "${SRC}"
if [[ "$(uname -m)" == "aarch64" && -z "${CC:-}" ]]; then
  # The SDK README asks for an explicit compiler on aarch64.
  export CC=/usr/bin/gcc CXX=/usr/bin/g++
fi
"${CONAN}" install . -s build_type=Release -s compiler.cppstd=17 -b missing -of build

cmake -S . -B build -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE="${SRC}/build/conan_toolchain.cmake" \
  -DCMAKE_POLICY_DEFAULT_CMP0091=NEW \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_EXAMPLES=OFF
cmake --build build -j "${JOBS}"
${SUDO} cmake --install build

echo "rby1-sdk ${VERSION} installed to ${PREFIX}"
echo "  export CMAKE_PREFIX_PATH=${PREFIX}:\${CMAKE_PREFIX_PATH:-}"
echo "  export LD_LIBRARY_PATH=${PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
