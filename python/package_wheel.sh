#!/usr/bin/env bash
# Build the manylinux wheel for the Python binding.
#
# Prerequisites: a completed build with the binding enabled, e.g.
#   cmake -S . -B build_wheel -DCMAKE_BUILD_TYPE=Release \
#         -DDUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON \
#         -DDUAL_ARM_V2_2_SDK_BUILD_CPP_EXAMPLES=OFF \
#         -DDUAL_ARM_V2_2_SDK_BUILD_C_EXAMPLES=OFF
#   cmake --build build_wheel -j
#
# Usage: python/package_wheel.sh [version] [build_dir]
#   version   default 1.0.0 (same as the CMake project version)
#   build_dir default <repo>/build_wheel
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
VERSION="${1:-1.0.0}"
BUILD_DIR="${2:-$ROOT/build_wheel}"
STAGE="$ROOT/whl_build"
PKG="$STAGE/dual_arm_v2_2_sdk"

[ -d "$BUILD_DIR/python" ] || { echo "missing $BUILD_DIR/python, configure with -DDUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON" >&2; exit 1; }

EXT_SO="$(ls "$BUILD_DIR"/python/dual_arm_v2_2_sdk.cpython-*.so)"
SDK_SO="$(find "$BUILD_DIR/lib" -name 'libdual_arm_v2_2_control_sdk.so.*.*.*' -type f | head -1)"
[ -f "$SDK_SO" ] || { echo "libdual_arm_v2_2_control_sdk.so.<version> not found in $BUILD_DIR/lib" >&2; exit 1; }
# yaml-cpp is often archived into the SDK .so. Vendor it only when NEEDED.
YAML_NEEDED="$(readelf -d "$SDK_SO" | awk '/NEEDED/ && /yaml-cpp/ {gsub(/[\[\]]/, "", $NF); print $NF; exit}')"
YAML_SO=""
if [ -n "$YAML_NEEDED" ]; then
    YAML_SO="${YAML_CPP_SO:-$(ldconfig -p | awk -v name="$YAML_NEEDED" '$1 == name {print $NF; exit}')}"
    [ -n "$YAML_SO" ] && [ -f "$YAML_SO" ] || { echo "SDK needs $YAML_NEEDED but it was not found; set YAML_CPP_SO" >&2; exit 1; }
fi
SDK_SONAME="$(basename "$SDK_SO" | sed -E 's/(\.so\.[0-9]+).*/\1/')"

echo "== staging $PKG (version $VERSION)"
rm -rf "$STAGE"
mkdir -p "$PKG/libs"
cp "$HERE/packaging/pyproject.toml" "$HERE/packaging/MANIFEST.in" "$STAGE/"
cp "$HERE/README.md" "$STAGE/README.md"
cp "$HERE/packaging/dual_arm_v2_2_sdk/__init__.py" "$PKG/"
cp "$HERE/packaging/dual_arm_v2_2_sdk/libs/__init__.py" "$PKG/libs/"
sed -i "s/^version = \".*\"/version = \"$VERSION\"/" "$STAGE/pyproject.toml"

cp "$EXT_SO" "$PKG/"
# Ship one file under the SONAME. A wheel stores symlinks as full copies.
cp "$SDK_SO" "$PKG/libs/$SDK_SONAME"
if [ -n "$YAML_SO" ]; then
    cp -L "$YAML_SO" "$PKG/libs/$(basename "$YAML_SO")"
else
    echo "== yaml-cpp is static; not vendoring libyaml-cpp"
fi
chmod 755 "$PKG"/*.so "$PKG"/libs/*.so*

# $ORIGIN/libs for the extension; $ORIGIN for the SDK lib so the vendored
# libyaml-cpp is found on machines that do not have it installed (RUNPATH is
# not inherited by dependency lookups). The C library is not part of this wheel.
echo "== patching rpath"
patchelf --set-rpath '$ORIGIN/libs' "$PKG/dual_arm_v2_2_sdk.cpython-"*.so
patchelf --set-rpath '$ORIGIN' "$PKG/libs/$SDK_SONAME"

echo "== building wheel"
cd "$STAGE"
rm -rf build dual_arm_v2_2_sdk.egg-info
python3 -m build --wheel --no-isolation >/dev/null

echo "== repairing wheel"
# Our own libs stay in libs/ (excluded from grafting) so the wheel keeps a
# single copy of each library; auditwheel only retags the platform.
WHEEL="$(ls dist/dual_arm_v2_2_sdk-"$VERSION"-*.whl)"
auditwheel repair \
    --exclude 'libdual_arm_v2_2_control_sdk*' \
    --exclude 'libyaml-cpp*' \
    -w wheelhouse "$WHEEL"

ls -l "$STAGE/wheelhouse"
