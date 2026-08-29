#!/usr/bin/env bash
if [ -n "$PIXI_PROJECT_ROOT" ]; then
    export ZENOH_SESSION_CONFIG_URI="${PIXI_PROJECT_ROOT}/config/zenoh_client_local.json5"
fi
