#!/usr/bin/env bash

docker build -t event_orin_drone -f docker/Dockerfile --ssh default --build-arg BASE_IMAGE=event-orin:r36.3.0 .
