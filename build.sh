#!/bin/bash -x
cmake -DCMAKE_BUILD_TYPE=Release -B build
make -j4 -C build
