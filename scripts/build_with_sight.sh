#!/usr/bin/env bash
# Builds the CT AR application inside a Sight source tree.
#
# Sight applications are built as part of Sight's own CMake project (like the tutorials and Sight Viewer), so this
# script clones Sight, links ../ct_ar into sight/app/ and builds only the "ct_ar" target and its dependencies.
#
# Environment variables:
#   SIGHT_DIR   Sight source directory          (default: ./sight next to this repository)
#   BUILD_DIR   build directory                 (default: $SIGHT_DIR-build)
#   SIGHT_REF   Sight git commit/branch/tag     (default: the commit the application was written against)
#   BUILD_TYPE  CMake build type                (default: Release)
#   INSTALL_DIR CMake install prefix, must be empty (default: $SIGHT_DIR-install)
#   PYTHON      Python used by CMake            (default: /usr/bin/python3, the one the system VTK was built with)
#
# Extra arguments are passed to the CMake configure step, e.g. ./build_with_sight.sh -DBLA_VENDOR=OpenBLAS
#
# Sight's dependencies (Qt6, VTK, ITK, OGRE, OpenCV, DCMTK, Boost, ...) must be installed first, see
# https://sight.pages.ircad.fr/sight-doc/Installation/index.html
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIGHT_DIR="${SIGHT_DIR:-${REPO_DIR}/sight}"
BUILD_DIR="${BUILD_DIR:-${SIGHT_DIR}-build}"
SIGHT_REF="${SIGHT_REF:-3a22f2038dba50bbafed6826b9c1ad0257e12a49}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
INSTALL_DIR="${INSTALL_DIR:-${SIGHT_DIR}-install}"
PYTHON="${PYTHON:-/usr/bin/python3}"

if [[ ! -d "${SIGHT_DIR}/.git" ]]; then
    git clone https://github.com/IRCAD/sight.git "${SIGHT_DIR}"
fi

git -C "${SIGHT_DIR}" fetch --quiet origin "${SIGHT_REF}" || true
git -C "${SIGHT_DIR}" checkout --quiet "${SIGHT_REF}"

# Register the application in Sight's app/ folder.
ln -sfn "${REPO_DIR}/ct_ar" "${SIGHT_DIR}/app/ct_ar"
if ! grep -q "add_subdirectory(ct_ar)" "${SIGHT_DIR}/app/CMakeLists.txt"; then
    echo "add_subdirectory(ct_ar)" >> "${SIGHT_DIR}/app/CMakeLists.txt"
fi

GENERATOR=()
if command -v ninja > /dev/null; then
    GENERATOR=(-G Ninja)
fi

cmake -S "${SIGHT_DIR}" -B "${BUILD_DIR}" "${GENERATOR[@]}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DSIGHT_BUILD_TESTS=OFF \
    -DSIGHT_BUILD_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DPython3_EXECUTABLE="${PYTHON}" \
    "$@"

cmake --build "${BUILD_DIR}" --target ct_ar --parallel

echo
echo "Done. Run the application with:"
echo "    ${BUILD_DIR}/bin/ct_ar"
