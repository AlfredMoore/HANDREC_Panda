#!/usr/bin/bash

# change the owner of cargo
sudo chown -R $(whoami):$(whoami) $CARGO_HOME

# install dependencies
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -r -y