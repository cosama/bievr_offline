#!/usr/bin/env bash
# Build the BIEVR-LIO offline image from a minimal context (bridge core plus the
# pinned upstream estimator sources) so the repository's datasets never enter
# the context.
#
#   frameworks/bievr/docker/build.sh [image-tag]
#
# Default tag: ghcr.io/cosama/bievr_lio_offline:latest (the `rot bievr` default;
# override at run time with ROS_OFFLINE_BIEVR_IMAGE).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
tag="${1:-ghcr.io/cosama/bievr_lio_offline:latest}"
engine="${CONTAINER_ENGINE:-docker}"

tar -C "${repo_root}" -c \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' \
    --exclude='frameworks/bievr/core/build' \
    --transform='s,^frameworks/bievr/docker/Dockerfile$,Dockerfile,' \
    frameworks/bievr/docker/Dockerfile \
    frameworks/bievr/core \
    upstream/BIEVR-LIO/BIEVR/include \
    upstream/BIEVR-LIO/BIEVR/src \
  | "${engine}" build -t "${tag}" -
