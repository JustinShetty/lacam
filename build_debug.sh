#!/bin/bash
cmake -DCMAKE_BUILD_TYPE=Debug -B build
make -j4 -C build
