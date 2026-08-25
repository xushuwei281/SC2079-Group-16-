# Workspace setup (pixi)

This ROS 2 Jazzy workspace uses [Pixi](https://pixi.sh) for dependency
management. Everything below assumes you have `pixi` installed and nothing
else — no system ROS, no `apt install ros-*`.

## Pick your environment first

The ROS 2 graph runs across **two hosts** (see
[`ARCHITECTURE.md`](ARCHITECTURE.md)), so this workspace has **two
environments over one source tree**:

| Environment | Runs on | Contains | Doesn't contain |
|---|---|---|---|
| `pi` | Raspberry Pi 4B (`linux-aarch64`) | rclpy, pyserial, tf2, image transport | RViz2, YOLO/torch, CUDA, xacro |
| `pc` | laptop/PC (`linux-64`) or Mac (`osx-arm64`) | all of the above **plus** RViz2, cv_bridge, vision_msgs, xacro, ultralytics | — |

Concretely, `pi` resolves to **544 packages**; `pc` resolves to 672. The
difference is torch, torchvision, the full NVIDIA CUDA stack, and the Qt/OGRE
tree behind RViz2 — several GB the Pi has no use for, since it runs no
inference and no visualization.

You must always name the environment. `pixi run build` on its own errors out
and tells you so:

```
Error: × the task 'build' is ambiguous
  help: These environments provide the task 'build': pi, pc
```

That's deliberate. There is no usable `default` environment, so it's not
possible to silently build against the wrong dependency set.

## On the Raspberry Pi

```bash
pixi install -e pi        # first time, or after pixi.toml/pixi.lock changes
pixi run -e pi build      # colcon build --packages-up-to mdp_hardware_bridge mdp_bringup
pixi shell -e pi          # interactive shell with ROS + the built workspace sourced
```

## On a laptop/PC or Mac

```bash
pixi install -e pc
pixi run -e pc build      # colcon build (whole workspace)
pixi shell -e pc
```

## Other tasks

```bash
pixi run -e pi test       # or -e pc
pixi run -e pc clean      # rm -rf build install log
```

After the first build, pixi sources `install/setup.bash` on activation
automatically — `ros2 pkg list` will show `mdp_interfaces`,
`mdp_hardware_bridge`, and `mdp_bringup` without any manual sourcing.

`ROS_DOMAIN_ID=16` is set by activation in **both** environments, from a
single definition in `[feature.common.activation.env]`. Don't set it anywhere
else; if the Pi and the PC ever disagree on this number they simply won't see
each other, with no error message from either side.

## Adding a dependency — which section?

`pixi.toml` has three dependency blocks. Putting something in the wrong one
is the main way this setup degrades, so:

| Add it to | When |
|---|---|
| `[feature.common.dependencies]` | A node that **physically runs on the Pi** needs it (or both hosts do). This lands on the robot — keep it lean. |
| `[feature.pi.dependencies]` | Pi-only hardware packages, e.g. the libcamera camera driver once that's chosen. |
| `[feature.pc.dependencies]` / `[feature.pc.pypi-dependencies]` | Perception, planning, or visualization. Anything heavy defaults here. |

Then `pixi lock` and commit the updated `pixi.lock`.

Two packages land on the Pi that look like they shouldn't:
`cv_bridge` (pulled in by `compressed_image_transport`) and
`robot_state_publisher` (part of the `ros_base` variant). Both are transitive
and unavoidable, and both are small. Not a mistake — don't "fix" it.

## Adding a package that runs on the Pi

The `pi` build task names its packages explicitly:

```toml
build = { cmd = "colcon build --packages-up-to mdp_hardware_bridge mdp_bringup" }
```

When you scaffold `mdp_android_bridge` or the camera driver, **add it to that
list**. Otherwise it builds fine on your laptop under `-e pc` and silently
never gets built on the robot.

## Why one workspace instead of two

The obvious alternative is a separate `ros2_ws` per host. We don't do that
because `mdp_interfaces` is shared, and ROS 2 hashes message definitions into
the type name. Two copies that drift by one field produce nodes that build
clean, start clean, discover each other over DDS — and then never deliver a
message, with no error pointing at the cause. One source tree makes that
failure class impossible; the environment split gets the dependency savings
without it.

Unused source on the Pi costs ~200KB of Python that never executes, since a
ROS node only runs if something launches it.

## Common issues

### `environment 'pi' is not available on platform osx-arm64`

Expected. The `pi` environment is locked for `linux-aarch64` only. Use
`-e pc` on your laptop.

### `ros2` commands not found

Run through pixi: `pixi run -e <env> <command>`, or `pixi shell -e <env>`.

### Build fails after moving or renaming the workspace

`colcon` bakes absolute paths into `install/`. Run `pixi run -e <env> clean`
and build again.

### Active ROS environment conflict

If you have a system ROS sourced in `~/.bashrc` / `~/.zshrc`
(`source /opt/ros/jazzy/setup.bash`), comment it out and restart your shell.
Pixi manages the ROS environment; two of them at once will not work.

## Learn more

- [Pixi documentation](https://pixi.sh) · [multi-environment docs](https://pixi.sh/latest/workspace/multi_environment/)
- [RoboStack](https://robostack.github.io/)
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
