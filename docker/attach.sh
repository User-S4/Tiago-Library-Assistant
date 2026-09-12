#!/bin/bash
# Open a shell in the running container (sources workspaces via entrypoint)
cd "$(dirname "$0")" && docker compose exec erc /entrypoint.sh bash