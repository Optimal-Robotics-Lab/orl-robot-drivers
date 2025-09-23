# ROS2 API for various ORL Robots
Bazel build for various ORL Robots ROS2 Interface that allows importing via `bazel_dep`.

## 🚧 Warning 🚧
This repository is a work in progress, will be broken, and most likely significantly change. I would not recommend relying on this repository.

## Usage
Add the following to your `MODULE.bazel` file:

```python
bazel_dep(name = "orl-robot-drivers")
git_override(
    module_name = "orl-robot-drivers",
    remote = "git@github.com:Optimal-Robotics-Lab/orl-robot-drivers.git",
    commit = "[insert the latest commit hash]",
)
```
