# Docker

The build context must be the BIEVR framework root because the image needs
both `core/` and the pinned source under `upstream/`.

From the `slam_benchmark` repository root:

```bash
frameworks/bievr/docker/build.sh
```

Or manually:

```bash
docker build \
  -f frameworks/bievr/docker/Dockerfile \
  -t bievr-offline \
  frameworks/bievr
```

Smoke test:

```bash
docker run --rm bievr-offline
```
