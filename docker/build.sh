#!/usr/bin/env bash
# Build the BIEVR offline image from a minimal context (bridge core plus the
# pinned upstream estimator sources) so the repository's datasets never enter
# the context.
#
#   docker/build.sh [image-tag]
#
# Default tag: ghcr.io/cosama/bievr_offline:latest (the `rot bievr` default;
# override at run time with ROS_OFFLINE_BIEVR_IMAGE).
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
tag="${1:-ghcr.io/cosama/bievr_offline:latest}"
if [ "$#" -gt 0 ]; then shift; fi
engine="${CONTAINER_ENGINE:-docker}"

tar -C "${repo_root}" -c \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' \
    --exclude='core/build' \
    --exclude='core/.venv' \
    --transform='s,^docker/Dockerfile$,Dockerfile,' \
    docker/Dockerfile \
    core \
    upstream/BIEVR-LIO-SLAM/BIEVR/include \
    upstream/BIEVR-LIO-SLAM/BIEVR/src \
    upstream/BIEVR-LIO-SLAM/modules \
  | "${engine}" build -t "${tag}" "$@" -
