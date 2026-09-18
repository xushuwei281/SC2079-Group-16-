#!/usr/bin/env bash
# colcon build on this Mac's toolchain leaves mdp_interfaces' three Python
# typesupport extensions with only ONE rpath (the conda env's own lib/),
# missing one back to their own package's lib/ dir where
# libmdp_interfaces__rosidl_generator_py.dylib actually lives. Any node that
# imports an mdp_interfaces message/service type then fails with
# `Library not loaded: @rpath/libmdp_interfaces__rosidl_generator_py.dylib`.
# Re-run after every build that touches mdp_interfaces -- colcon regenerates
# these files from scratch each time, wiping this fix.
#
# Must use the conda env's OWN install_name_tool (cctools-port), not the
# system one at /usr/bin: only the conda one auto-regenerates the ad-hoc
# code signature that modifying rpaths invalidates. Skipping that step gets
# the modified binary SIGKILLed (exit 137) the moment anything tries to
# dlopen it.
#
# No-op on Linux: colcon's rpath handling there already works correctly.
set -euo pipefail

if [ "$(uname)" != "Darwin" ]; then
    exit 0
fi

cd "$(dirname "$0")/.."

INSTALL_NAME_TOOL=".pixi/envs/pc/bin/install_name_tool"
if [ ! -x "$INSTALL_NAME_TOOL" ]; then
    exit 0  # pc env not installed yet; nothing to fix
fi

RPATH="@loader_path/../../../../lib"
SITE_PACKAGES="install/mdp_interfaces/lib/python3.12/site-packages/mdp_interfaces"

for so in "$SITE_PACKAGES"/mdp_interfaces_s__rosidl_typesupport_c.so \
          "$SITE_PACKAGES"/mdp_interfaces_s__rosidl_typesupport_introspection_c.so \
          "$SITE_PACKAGES"/mdp_interfaces_s__rosidl_typesupport_fastrtps_c.so; do
    if [ -f "$so" ] && ! otool -l "$so" | grep -q "$RPATH"; then
        "$INSTALL_NAME_TOOL" -add_rpath "$RPATH" "$so"
    fi
done
