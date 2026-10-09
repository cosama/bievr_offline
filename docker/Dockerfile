ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION} AS builder

ENV DEBIAN_FRONTEND=noninteractive

# The empty libCppUnitLite.a stub: libgtsam-dev's CMake exports reference
# GTSAM's unit-test library, which the package does not ship, so
# find_package(GTSAM) fails without it.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        libceres-dev \
        libeigen3-dev \
        libgtsam-dev \
        libmetis-dev \
        libpcl-dev \
        libtbb-dev \
        libyaml-cpp-dev \
        ninja-build \
        patch \
        python3 \
        python3-dev \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/* \
    && ar cr /usr/lib/x86_64-linux-gnu/libCppUnitLite.a

WORKDIR /src
COPY core /src/core
COPY upstream /src/upstream

RUN python3 -m venv /opt/build-venv \
    && /opt/build-venv/bin/pip install --no-cache-dir --upgrade pip wheel \
    && /opt/build-venv/bin/pip wheel --no-cache-dir --wheel-dir /wheelhouse \
        ./core "rosbags>=0.11,!=0.11.6" "ruamel.yaml"

FROM ubuntu:${UBUNTU_VERSION} AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/opt/bievr/bin:${PATH}
ENV PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libceres4t64 \
        libgtsam4 \
        libmetis5 \
        libpcl-common1.14 \
        libpcl-filters1.14 \
        libpcl-io1.14 \
        libpcl-registration1.14 \
        libtbb12 \
        libyaml-cpp0.8 \
        python3 \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /wheelhouse /wheelhouse

RUN python3 -m venv /opt/bievr \
    && /opt/bievr/bin/pip install --no-cache-dir --no-index --find-links=/wheelhouse \
        bievr-offline rosbags ruamel.yaml \
    && rm -rf /wheelhouse

WORKDIR /data

CMD ["python", "-c", "import bievr, rosbags; from ruamel.yaml import YAML; print('bievr-offline ready')"]
