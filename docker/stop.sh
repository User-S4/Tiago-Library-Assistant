#!/bin/bash
# Stop the ERC simulation container
cd "$(dirname "$0")" && docker compose down
