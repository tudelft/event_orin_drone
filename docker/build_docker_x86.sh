#!/usr/bin/env bash

docker build -t event_orin_drone -f docker/Dockerfile --ssh default .
