#!/usr/bin/env bash

jetson-containers build --name event-orin cuda:12.2 pytorch:2.4 openai-triton:3.0.0
