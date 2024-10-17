# Setup 

### 0. Real-time Kernel and Docker 
 * Possible required packages for Real-time Kernel
 ```bash
 sudo apt-get install --yes \
    build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
sudo apt-get install --yes \
    flex bison libncurses-dev debhelper
 ```
 * how to install [real-time kernel](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel) for libfranka
 * docker [instruction](../ros-devcontainer/README.md)
    * After docker installation, enter a temperary container
    ```bash
    cd ros-devcontainer
    ./enter-container.sh
    ```

### 1. Install [libfranka and frankaros](https://frankaemika.github.io/docs/installation_linux.html)
TLDR:
* Build from binary (It has been done in the dockerfile): 
```bash
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```
 * Build from [source](https://frankaemika.github.io/docs/installation_linux.html)

### 2. Build the relaxed_IK
#### relaxed_IK_core
TLDR:
 * cd into the [relaxed_ik_core directory](../relaxed_ik_core)
 * Change the owner of cargo for permission to WRITE
 ```bash
 sudo chown -R $(whoami):$(whoami) $CARGO_HOME
 ```
 * complile the repo
 ```bash
 cargo build
 ```
 * test with demo
 ```bash
 cargo run --bin relaxed_ik_bin
 ```
 * Place your robot's URDF under [here](../relaxed_ik_core/configs/urdfs/), which already contains `panda.urdf`.
 * Use your own robot setting. Make a setting file. [Examples](../relaxed_ik_core/configs/example_settings)
#### relaxed_IK_ros1
 TLDR:
 * Build the relaxed_IK_ros1 package.
 ```bash
 cd <repo_directory>
 catkin build
 ```
 * Test if relaxed_IK working well
 ```bash
 source devel/setup.zsh for ZSH shell or devel/setup.bash for BASH with a simple demo
 ```
 Test
 ```bash
roslaunch relaxed_ik_ros1 demo.launch
 ```