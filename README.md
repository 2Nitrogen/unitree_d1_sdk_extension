# unitree_arm — Unitree D1 arm SDK

The Unitree D1 servo-arm SDK is not covered by [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) and is shipped as a `.zip` file of example programs that each recompile the generated DDS type sources - [D1 arm development guide coduments](https://support.unitree.com/home/en/developer/D1Arm_services).
This repo packages the same material as a proper CMake package that installs
next to `unitree_sdk2`, so a C++ project can use the Go2 and the D1 through one
consistent mechanism.

## Install

### 1. Install `unitree_sdk2`

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<prefix>   # prefix '/opt/unitree_robotics' recommended
sudo make install
```

### 2. Install d1_sdk

```bash
git clone https://github.com/2Nitrogen/unitree_d1_sdk_extension.git
cd unitree_d1_sdk_extension
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=<prefix>
cmake --build build -j
sudo cmake --install build
```

### 3. Check installed files

Installed files, all under their own `unitree_arm/` subtree so that
reinstalling `unitree_sdk2` can never collide with them:

```
<prefix>/include/unitree_arm/idl/{ArmString_,PubServoInfo_,SetServoAngle_,SetServoDumping_}.hpp
<prefix>/include/unitree_arm/dds_wrapper/d1/{defines,d1_protocol,d1_pub,d1_sub,d1}.h
<prefix>/include/unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h
<prefix>/lib/libunitree_arm.a
<prefix>/lib/cmake/unitree_arm/{unitree_armConfig,unitree_armConfigVersion,unitree_armTargets}.cmake
```

`d1_mujoco_bridge.h` is the one that matters for simulation. A stale install can
leave the cmake config in place without it, which satisfies `find_package` and
then fails at `#include` -- so check for that file specifically after upgrading.

### 4. Wire up the simulator (optional)

To drive the arm inside [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco):

```bash
./setup.sh install
```

See [Simulation (unitree_mujoco)](#simulation-unitree_mujoco) for what it changes
and how to do it by hand.

## Use

```cmake
find_package(unitree_arm REQUIRED)      # pulls in unitree_sdk2 itself
target_link_libraries(my_app unitree_arm)
```

```cpp
#include <unitree_arm/dds_wrapper/d1/d1.h>
using namespace unitree::robot;

ChannelFactory::Instance()->Init(0, "eth0");

// host side: drive the arm
d1::publisher::ArmCommand cmd;
cmd.SendAllJointAngle({0, -60, 60, 0, 30, 0, 0});   // degrees, J0..J6
cmd.SendZero();

// read what it reports
d1::subscription::ServoAngle angle;
angle.wait_for_connection();
printf("J1 = %.2f deg\n", angle.angles[1]);
```

The publisher/subscription classes mirror `unitree/dds_wrapper/robots/go2`, so
`unitree::robot::d1::publisher::ArmCommand` sits alongside
`unitree::robot::go2::publisher::LowCmd` and behaves the same way.

Standing in for the arm (a simulator, a test rig) is the same API from the
other side:

```cpp
d1::subscription::ArmCommand cmd;      // decoded into cmd.command
d1::publisher::ServoAngle    angle;    // current_servo_angle, 10 Hz
d1::publisher::ArmFeedback   fb;       // rt/arm_Feedback
```

## Simulation (unitree_mujoco)

`unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h` serves a D1 that lives inside a
MuJoCo model: it subscribes to `rt/arm_Command`, drives the arm's actuators, and
reports angles on `current_servo_angle` and `rt/arm_Feedback` at 10 Hz. Control
code written for the real arm then drives the simulator unchanged.

It is a header-only add-on. Including it pulls in MuJoCo, so projects that do
not simulate never pay for it and `libunitree_arm` keeps no MuJoCo dependency.

Wiring it into [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
takes four edits and two assets a fresh clone does not carry. All of the edits
are no-ops on a model without arm actuators, so every other robot keeps its
original behaviour.

### Automatic setup

```bash
./setup.sh check      # what would change, and what is already wired
./setup.sh install    # apply it
./setup.sh verify     # build, then confirm the bridge actually starts
```

`setup.sh` finds unitree_mujoco at `$UNITREE_MUJOCO_DIR`, `../unitree_mujoco` or
`~/unitree_mujoco`; pass `--repo PATH` to point it elsewhere, and `-n` to see the
diff without writing anything. It is idempotent, backs the four originals up
under `$XDG_STATE_HOME/d1_sdk_extension/backups/` and undoes itself with
`./setup.sh revert`.

Each edit is declared as *"this one line must appear exactly once; put this
block after it"*, so an upstream revision that reworks the code around an anchor
still gets wired correctly. If an anchor is missing or appears more than once the
script names it and writes nothing at all, rather than half-applying -- a
half-wired tree still compiles cleanly, which is the failure this script exists
to prevent. Apply the edit by hand from **Manual setup** below when that
happens.

Only `main.resolve_idl` is revision-sensitive: it rewrites a line that arrived
upstream in `0244cc8` (AS2 support), so on an older checkout the script warns and
carries on. That is safe -- go2-d1 has 19 actuators and the `unitree_go` IDL
allows 20, so IDL selection is already correct without it.

`setup.sh` needs `python3` (standard library only) for the multi-line edits;
unitree_mujoco already ships `simulate_python/`, so any machine that runs the
simulator has it.

<details>
<summary>The nine edits, by anchor</summary>

| hunk | file | anchor line (must be unique) |
|---|---|---|
| `yaml.robot` | `simulate/config.yaml` | `robot:` |
| `yaml.domain_id` | `simulate/config.yaml` | `domain_id:` |
| `cmake.find_package` | `simulate/CMakeLists.txt` | `find_package(unitree_sdk2 REQUIRED)` |
| `cmake.link_lib` | `simulate/CMakeLists.txt` | `  unitree_sdk2` (in `link_libraries`) |
| `bridge.include` | `simulate/src/unitree_sdk2_bridge.h` | `#include <unitree/idl/hg/IMUState_.hpp>` |
| `bridge.member` | `simulate/src/unitree_sdk2_bridge.h` | `    int dim_motor_sensor_ = 0;` |
| `bridge.detect` | `simulate/src/unitree_sdk2_bridge.h` | `        num_motor_ = mj_model_->nu;` |
| `main.start_bridge` | `simulate/src/main.cc` | `  interface->start();` |
| `main.resolve_idl` | `simulate/src/main.cc` | `    idl_type = param::ResolveIdlType(param::config.robot, m->nu, param::config.idl_type);` |

</details>

### Manual setup

`simulate/config.yaml`

```yaml
robot: "go2-d1"
domain_id: 0      # the stock example programs hardcode domain 0
```

`simulate/CMakeLists.txt`

```cmake
find_package(unitree_arm REQUIRED)
# ... and add unitree_arm to the link_libraries() list
```

`simulate/src/unitree_sdk2_bridge.h`

```cpp
#include <unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h>

// member of UnitreeSDK2BridgeBase
unitree::robot::d1::MujocoArmLayout arm_;

// first thing in _check_sensor(), before dim_motor_sensor_ is computed:
// hands the arm's actuators to the D1 protocol so LowCmd stops at the legs
arm_ = unitree::robot::d1::MujocoArmLayout::Detect(mj_model_);
if (arm_.valid()) num_motor_ = arm_.base;
```

`simulate/src/main.cc`

```cpp
// after interface->start()
auto arm_bridge = unitree::robot::d1::StartMujocoArmBridge(m, d);
```

Detection keys off an actuator named `d1_J0` rather than the robot's name, so
any scene that mounts a D1 is picked up automatically. The arm shares the
`ChannelFactory` the simulator already initialises, which means it answers on
the same DDS domain as the robot -- set `domain_id: 0` in `config.yaml` if you
want the stock example programs, which hardcode domain 0, to reach it.

Without the `num_motor_` edit a Go2 `LowCmd` would write past the legs and zero
the arm's servos on every control cycle.

Two assets also have to be in place, and neither travels with a `git clone` of
unitree_mujoco -- `setup.sh install` provisions both:

- `unitree_robots/go2-d1/` -- unpack `robot_model/Go2_D1_mjcf.zip` and rename its
  `Go2_D1_mjcf/` root to `go2-d1/`. The directory name has to match `robot:` in
  `config.yaml`, because the simulator resolves the scene as
  `unitree_robots/<robot>/<robot_scene>`. `unitree_robots/` is not gitignored, so
  this shows up as untracked -- don't `git clean` it away.
- `simulate/mujoco` -- a symlink to a MuJoCo release root (the one holding
  `include/`, `lib/` and `simulate/`; a pip-installed `mujoco` has no `simulate/`
  sources and will not do). Already gitignored upstream.

### Verifying

Whatever the build said, the simulator must print this at startup:

```
[d1] serving the arm on rt/arm_Command: ctrl[12..18] at 10 Hz
```

If the line is absent nothing is subscribed to `rt/arm_Command`, and control
programs will simply be ignored. The range matters too: `12..18` is the Go2's
twelve leg actuators followed by `d1_J0..d1_J6`. Check also that the Unitree side
reports **12** motors rather than 19 -- 19 means the `num_motor_ = arm_.base`
edit did not take effect.

Keep only one simulator instance on a DDS domain. Two of them answering on the
same domain interleave their `current_servo_angle` publications, which looks
exactly like a miscalibrated gripper. `-i <domain_id>` puts an instance on its
own domain.

## Examples

`examples/` holds Unitree's six original programs unchanged. They are the package's own smoke test — if they build, the install is self-contained.
Build them with `-DUNITREE_ARM_BUILD_EXAMPLES=ON` (the default);
binaries land in `build/examples/`.

Each takes a network interface as its first argument and uses DDS domain 0:

```bash
./build/examples/get_arm_joint_angle lo
./build/examples/multiple_joint_angle_control lo
```

`joint_angle_control_interactive` is this package's own addition. The stock
`joint_angle_control` hardcodes its target, so a different joint or angle costs
an edit and a rebuild; this one takes them at runtime, either as arguments or
line by line from the terminal, and prints the angles the arm reports back:

```bash
./build/examples/joint_angle_control_interactive lo 1 90 500  # J1 -> 90 deg over 500 ms
./build/examples/joint_angle_control_interactive lo           # interactive; h for help
```

Angles are degrees, joint ids run J0..J6 (J6 is the gripper). In interactive
mode a line is `<id> <angle_deg> [delay_ms]`; `a` prints the current angles, `z`
returns the arm to zero, `q` quits. Being ordinary stdin, it scripts too:
`printf '1 90\n2 -30\nz\n' | ./joint_angle_control_interactive lo`.

## Layout

```
include/unitree_arm/idl/          Cyclone DDS IDL types, as shipped by Unitree
include/unitree_arm/dds_wrapper/  protocol constants, codec, pub/sub facades,
                                  MuJoCo arm bridge (header-only)
src/idl/                          generated CDR descriptors (must be compiled)
src/d1_protocol.cpp               codec implementation
examples/                         Unitree's original programs, plus the
                                  interactive single-joint driver
```

The `.hpp`/`.cpp` pairs under `idl/` are pre-generated by Cyclone DDS's IDL
compiler.

## PLACEHOLDER

D1 sdk to be used in Mujoco, proper bridge is further required ... development ongoing
