#!/usr/bin/env bash

# install dependencies
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -r -y