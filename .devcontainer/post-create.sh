#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${HOME}/unified_robot_interface"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
INSTALL_PREFIX="${PROJECT_DIR}/build/install"
EXTRA_DEPS_PREFIX="${HOME}/.local/unified_robot_interface-deps"

BUILD_JOBS="${BUILD_JOBS:-4}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"

echo "==> Loading ROS ${ROS_DISTRO} environment"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "==> Building URI with ${BUILD_JOBS} jobs"

rm -rf "${PROJECT_DIR}/build"

cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="${EXTRA_DEPS_PREFIX}" \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"

cmake --build "${PROJECT_DIR}/build" --parallel "${BUILD_JOBS}"

cmake --install "${PROJECT_DIR}/build"

echo "==> Setting up environment"
ENV_FILE="${HOME}/.unified_robot_interface_env"
cat > "${ENV_FILE}" <<EOF
source "/opt/ros/${ROS_DISTRO}/setup.bash"
export PATH="${PROJECT_DIR}/build/bin:\${PATH}"
export CMAKE_PREFIX_PATH="${INSTALL_PREFIX}:${EXTRA_DEPS_PREFIX}:\${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:${EXTRA_DEPS_PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
EOF

ENV_SOURCE_LINE='source "$HOME/.unified_robot_interface_env"'

for STARTUP_FILE in "${HOME}/.bashrc" "${HOME}/.profile"; do
  if ! grep -qxF "${ENV_SOURCE_LINE}" "${STARTUP_FILE}" 2>/dev/null; then
    printf '\n%s\n' "${ENV_SOURCE_LINE}" >> "${STARTUP_FILE}"
  fi
done

echo "==> Verifying shell startup configuration"

grep -qxF "${ENV_SOURCE_LINE}" "${HOME}/.bashrc"
grep -qxF "${ENV_SOURCE_LINE}" "${HOME}/.profile"

test -x "${PROJECT_DIR}/build/bin/uri"

echo "==> Done. The environment will be loaded automatically in new Bash sessions."
