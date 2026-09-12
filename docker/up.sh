#!/bin/bash
xhost +local:docker 2>/dev/null
cd "$(dirname "$0")"
docker compose down 2>/dev/null
docker rm -f erc_sim 2>/dev/null
COMPOSE="-f docker-compose.yml"

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
   && { docker info 2>/dev/null | grep -qi nvidia || command -v nvidia-ctk >/dev/null 2>&1; }; then
    echo "[up] NVIDIA GPU + container toolkit detected — enabling GPU passthrough"
    COMPOSE="$COMPOSE -f docker-compose.gpu.yml"
else
    echo "[up] No usable NVIDIA runtime — falling back to software/integrated rendering"
fi

ARGS=()
BUILD=0
for arg in "$@"; do
    if [ "$arg" = "--build" ]; then
        BUILD=1
    else
        ARGS+=("$arg")
    fi
done

if [ "$BUILD" -eq 1 ]; then
    echo "[up] Pulling latest base image"
    docker pull osrf/ros:humble-desktop
    echo "[up] Rebuilding image from scratch"
    docker compose $COMPOSE build --no-cache
fi

docker compose $COMPOSE up -d "${ARGS[@]}"
echo "Container running. Attach with: ./docker/attach.sh"
echo "Rebuild image:        ./docker/up.sh --build"
echo "Force GPU manually:   docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d"