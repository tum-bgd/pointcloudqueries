#!/bin/bash

# this is supposed to build manylinux wheels
docker run -it -rm  -v $PWD:/io quay.io/pypa/manylinux2014_x86_64
# run build.sh in container
