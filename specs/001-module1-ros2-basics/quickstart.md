# Quickstart Guide: Module 1 ROS 2 Basics

## Target Audience

This guide is for students who want to follow the "Module 1: ROS 2 — The Robotic Nervous System" tutorial.

## Prerequisites

Before you begin, you must have the following software installed and configured on your system. All examples have been tested on this specific stack.

-   **Operating System**: Ubuntu 22.04 (Jammy Jellyfish)
-   **ROS 2 Version**: ROS 2 Humble Hawksbill (Desktop Install).

You can verify your ROS 2 installation by running:

```bash
printenv AMENT_PREFIX_PATH
```

This should show a path related to `/opt/ros/humble`.

## Setup

### 1. Create a Colcon Workspace

A "workspace" is a directory where you will build and run the tutorial's code.

```bash
# Create a directory for your ROS 2 workspace
mkdir -p ~/ros2_ws/src

# Navigate into the source directory
cd ~/ros2_ws/src
```

### 2. Get the Example Code

The example code for this module is located in the `examples/ros2_basics` directory of this project. Copy this directory into your workspace's `src` folder.

```bash
# From your workspace's src directory
# Replace <path_to_project> with the actual path to this project
cp -r <path_to_project>/examples/ros2_basics .
```

### 3. Build the Examples

Navigate to the root of your workspace (`~/ros2_ws`) and use `colcon` to build the example package.

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_basics_examples
```

If the build is successful, you will see a message indicating the number of packages finished.

## Running Your First Example

After a successful build, you need to source the workspace's setup file to make the executables available in your terminal.

### 1. Source the Workspace

In every new terminal you open, you must run this command from the root of your workspace (`~/ros2_ws`):

```bash
source install/setup.bash
```

### 2. Run the Publisher/Subscriber Example

This example demonstrates the core "Topic" communication pattern in ROS 2.

**Step A: Open a new terminal and run the subscriber.**

The subscriber node will wait for messages to be published on the `/chatter` topic.

```bash
# Remember to source your workspace first!
source ~/ros2_ws/install/setup.bash

ros2 run ros2_basics_examples simple_subscriber
```

You will see a message like `[INFO] [simple_subscriber]: Subscribing to topic '/chatter'`.

**Step B: Open a second terminal and run the publisher.**

The publisher node will send messages to the `/chatter` topic.

```bash
# Remember to source your workspace in this new terminal too!
source ~/ros2_ws/install/setup.bash

ros2 run ros2_basics_examples simple_publisher
```

You will see messages like `[INFO] [simple_publisher]: Publishing: 'Hello World: 0'`.

**Step C: Observe the output.**

Switch back to your first terminal (the subscriber). You will now see the messages being received:

`[INFO] [simple_subscriber]: I heard: 'Hello World: 0'`
`[INFO] [simple_subscriber]: I heard: 'Hello World: 1'`
`...`

Congratulations! You have successfully run your first ROS 2 program using the examples from this module.
