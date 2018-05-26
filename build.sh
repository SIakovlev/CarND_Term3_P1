#!/bin/bash

cd build/
cmake .. && make

echo Project is compiled

echo Running...

./path_planning
