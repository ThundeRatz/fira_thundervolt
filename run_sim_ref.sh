#!/bin/bash

xhost +local:docker

docker run --rm --privileged --net=host thunderatz/firasim:light
