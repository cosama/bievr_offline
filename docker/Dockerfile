ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION} AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        libceres-dev \
        libeigen3-dev \
        libtbb-dev \
        libyaml-cpp-dev \
        ninja-build \
        patch \
        python3 \
        python3-dev \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Same relative layout as the repository: core/CMakeLists.txt resolves the
# pinned upstream at ../../../upstream/BIEVR-LIO. See build.sh.
WORKDIR /src
COPY frameworks/bievr/core /src/frameworks/bievr/core
COPY upstream/BIEVR-LIO /src/upstream/BIEVR-LIO

RUN python3 -m venv /opt/build-venv \
    && /opt/build-venv/bin/pip install --no-cache-dir --upgrade pip wheel \
    && /opt/build-venv/bin/pip wheel --no-cache-dir --wheel-dir /wheelhouse \
        ./frameworks/bievr/core "rosbags>=0.11" "ruamel.yaml"

FROM ubuntu:${UBUNTU_VERSION} AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/opt/bievr/bin:${PATH}
ENV PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libceres4t64 \
        libtbb12 \
        libyaml-cpp0.8 \
        python3 \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /wheelhouse /wheelhouse

RUN python3 -m venv /opt/bievr \
    && /opt/bievr/bin/pip install --no-cache-dir --no-index --find-links=/wheelhouse \
        bievr-lio-offline rosbags ruamel.yaml \
    && rm -rf /wheelhouse

WORKDIR /data

CMD ["python", "-c", "import bievr_lio, rosbags; from ruamel.yaml import YAML; print('bievr-lio-offline ready')"]
