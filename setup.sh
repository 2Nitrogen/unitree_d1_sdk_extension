#!/usr/bin/env bash
#
# Wire the D1-550 MuJoCo arm bridge into a unitree_mujoco checkout.
#
# The bridge (include/unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h) is
# header-only, so unitree_mujoco compiles perfectly well without ever calling
# it -- which is exactly how a half-finished setup goes unnoticed: the build is
# clean and nothing subscribes to rt/arm_Command. This script applies all of
# the edits or none of them, and says which anchor it could not find.
#
# It edits four files in the unitree_mujoco tree and provisions two assets that
# a fresh clone does not carry. Run `./setup.sh help` for the full surface.
#
# Mechanism: every edit is declared as "this one line must appear exactly once;
# put this block after it". No unified diff, so upstream can rewrite the code
# around an anchor and the edit still lands; and no sed, because sed cannot
# assert a match count and would silently no-op on zero matches.

set -Eeuo pipefail
IFS=$'\n\t'

readonly SCRIPT_VERSION="1.0.0"
readonly REPO_A="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# unitree_mujoco revision these anchors were verified against. A mismatch is a
# warning, never an error: the anchor check is the real gate.
readonly KNOWN_GOOD_HEAD="4134cb5"

readonly ARM_PREFIX="/opt/unitree_robotics"
readonly ARM_CONFIG="$ARM_PREFIX/lib/cmake/unitree_arm/unitree_armConfig.cmake"
readonly ARM_HEADER="$ARM_PREFIX/include/unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h"

readonly MODEL_ZIP_NAME="Go2_D1_mjcf.zip"
readonly MODEL_ZIP_ROOT="Go2_D1_mjcf"
readonly ARM_ACTUATOR='name="d1_J0"'

# The four files the script may touch, relative to the unitree_mujoco root.
readonly TARGET_FILES=(
  "simulate/config.yaml"
  "simulate/CMakeLists.txt"
  "simulate/src/unitree_sdk2_bridge.h"
  "simulate/src/main.cc"
)

# ---------------------------------------------------------------- options ----

SUBCOMMAND="install"
REPO_B=""
MUJOCO_ROOT=""
MODEL_SRC=""
ROBOT_NAME="go2-d1"
DOMAIN_ID="0"
DRY_RUN=0
DO_EDITS=1
DO_PROVISION=1
ALLOW_MISSING=""
FORCE=0
ASSUME_YES=0
VERBOSE=0
RUN_SIM=1
FULL_VERIFY=0
BACKUP_ROOT="${XDG_STATE_HOME:-$HOME/.local/state}/d1_sdk_extension/backups"

STAGE=""
BACKUP_DIR=""

# ---------------------------------------------------------------- output -----

if [[ -t 2 ]]; then
  readonly C_RED=$'\033[31m' C_YEL=$'\033[33m' C_GRN=$'\033[32m'
  readonly C_DIM=$'\033[2m' C_BLD=$'\033[1m' C_OFF=$'\033[0m'
else
  readonly C_RED="" C_YEL="" C_GRN="" C_DIM="" C_BLD="" C_OFF=""
fi

info() { printf '%s[d1-setup]%s %s\n' "$C_DIM" "$C_OFF" "$*" >&2; }
step() { printf '%s[d1-setup]%s %s%s%s\n' "$C_DIM" "$C_OFF" "$C_BLD" "$*" "$C_OFF" >&2; }
ok()   { printf '%s[d1-setup]%s %sok%s    %s\n' "$C_DIM" "$C_OFF" "$C_GRN" "$C_OFF" "$*" >&2; }
warn() { printf '%s[d1-setup]%s %swarn%s  %s\n' "$C_DIM" "$C_OFF" "$C_YEL" "$C_OFF" "$*" >&2; }
die()  { printf '%s[d1-setup]%s %sFATAL%s %s\n' "$C_DIM" "$C_OFF" "$C_RED" "$C_OFF" "$*" >&2; exit "${2:-1}"; }
dbg()  { (( VERBOSE )) && printf '%s[d1-setup]       %s%s\n' "$C_DIM" "$*" "$C_OFF" >&2 || true; }

cleanup() { [[ -n "$STAGE" && -d "$STAGE" ]] && rm -rf "$STAGE" || true; }
trap cleanup EXIT

on_err() {
  local rc=$? line=$1
  printf '%s[d1-setup]%s %sFATAL%s unexpected failure at line %s (exit %s)\n' \
    "$C_DIM" "$C_OFF" "$C_RED" "$C_OFF" "$line" "$rc" >&2
  if [[ -n "$BACKUP_DIR" && -d "$BACKUP_DIR" ]]; then
    printf '           the four originals are saved at:\n             %s\n' "$BACKUP_DIR" >&2
    printf '           restore them with:  %s revert\n' "$0" >&2
  fi
  exit "$rc"
}
trap 'on_err $LINENO' ERR

# ------------------------------------------------------------------ help -----

cmd_help() {
  cat <<'EOF'
Wire the D1-550 MuJoCo arm bridge into a unitree_mujoco checkout.

USAGE
  ./setup.sh [SUBCOMMAND] [OPTIONS]

SUBCOMMANDS
  install    (default) preflight, apply the edits atomically, provision assets
  check      preflight and per-hunk state report; changes nothing
  status     one line per hunk (applied / pending / missing / ambiguous)
  revert     restore the four files from the newest backup
  verify     build the simulator and confirm the [d1] startup banner
  help       this text

OPTIONS
  --repo PATH         unitree_mujoco root
                      [$UNITREE_MUJOCO_DIR, ../unitree_mujoco, ~/unitree_mujoco]
  --mujoco PATH       MuJoCo root for the simulate/mujoco symlink
                      [$MUJOCO_DIR, newest ~/.mujoco/mujoco-*/mujoco-*]
  --model PATH        go2-d1 assets: a directory or a .zip
  --robot-name NAME   config.yaml robot value      (default: go2-d1)
  --domain-id N       config.yaml domain_id value  (default: 0)

  -n, --dry-run       stage and validate everything, print the diff, write nothing
  --edits-only        skip asset provisioning
  --provision-only    skip the source edits
  --allow-missing ID  downgrade a degradable hunk's missing anchor to a warning
                      (repeatable, comma-separated)
  --backup-dir DIR    default: $XDG_STATE_HOME/d1_sdk_extension/backups
  --force             proceed despite a partial or hand-edited state
  -y, --yes           non-interactive
  --no-run            verify: stop after the build, do not launch the simulator
  --full              verify: also run the DDS round-trip check
  -v, --verbose       show every hunk decision
  -h, --help          this text

EXIT CODES
  0   success, or already fully applied
  2   bad arguments
  10  check: not applied (clean, ready to install)
  11  partially applied -- needs `revert` or `--force`
  20  an anchor is missing or ambiguous; nothing was written
  30  preflight failed
  40  verify failed
EOF
}

# ------------------------------------------------------------ arg parsing ----

parse_args() {
  if [[ $# -gt 0 && "$1" != -* ]]; then
    case "$1" in
      install|check|status|revert|verify|help) SUBCOMMAND="$1"; shift ;;
      *) die "unknown subcommand '$1' (try: ./setup.sh help)" 2 ;;
    esac
  fi
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --repo)          REPO_B="${2:?--repo needs a path}"; shift 2 ;;
      --mujoco)        MUJOCO_ROOT="${2:?--mujoco needs a path}"; shift 2 ;;
      --model)         MODEL_SRC="${2:?--model needs a path}"; shift 2 ;;
      --robot-name)    ROBOT_NAME="${2:?--robot-name needs a value}"; shift 2 ;;
      --domain-id)     DOMAIN_ID="${2:?--domain-id needs a value}"; shift 2 ;;
      --backup-dir)    BACKUP_ROOT="${2:?--backup-dir needs a path}"; shift 2 ;;
      --allow-missing) ALLOW_MISSING="${ALLOW_MISSING:+$ALLOW_MISSING,}${2:?--allow-missing needs a hunk id}"; shift 2 ;;
      -n|--dry-run)    DRY_RUN=1; shift ;;
      --edits-only)    DO_PROVISION=0; shift ;;
      --provision-only) DO_EDITS=0; shift ;;
      --no-run)        RUN_SIM=0; shift ;;
      --full)          FULL_VERIFY=1; shift ;;
      --force)         FORCE=1; shift ;;
      -y|--yes)        ASSUME_YES=1; shift ;;
      -v|--verbose)    VERBOSE=1; shift ;;
      -h|--help)       cmd_help; exit 0 ;;
      *) die "unknown option '$1' (try: ./setup.sh help)" 2 ;;
    esac
  done
  [[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || die "--domain-id must be a non-negative integer, got '$DOMAIN_ID'" 2
  (( DO_EDITS || DO_PROVISION )) || die "--edits-only and --provision-only are mutually exclusive" 2
}

# -------------------------------------------------------------- discovery ----

resolve_repo_b() {
  local candidates=()
  [[ -n "$REPO_B" ]] && candidates+=("$REPO_B")
  [[ -n "${UNITREE_MUJOCO_DIR:-}" ]] && candidates+=("$UNITREE_MUJOCO_DIR")
  candidates+=("$(dirname "$REPO_A")/unitree_mujoco" "$HOME/unitree_mujoco")

  local c
  for c in "${candidates[@]}"; do
    if [[ -f "$c/simulate/src/main.cc" ]]; then
      REPO_B="$(cd "$c" && pwd)"
      dbg "unitree_mujoco: $REPO_B"
      return 0
    fi
  done
  die "could not find a unitree_mujoco checkout. Looked at:
$(printf '             %s\n' "${candidates[@]}")
           Pass --repo /path/to/unitree_mujoco." 30
}

resolve_mujoco_root() {
  if [[ -n "$MUJOCO_ROOT" ]]; then
    :
  elif [[ -n "${MUJOCO_DIR:-}" ]]; then
    MUJOCO_ROOT="$MUJOCO_DIR"
  else
    # Newest ~/.mujoco/mujoco-<ver>-linux-*/mujoco-<ver> by version sort.
    MUJOCO_ROOT="$(find "$HOME/.mujoco" -maxdepth 2 -mindepth 2 -type d -name 'mujoco-*' 2>/dev/null \
      | sort -V | tail -1)"
  fi
  [[ -n "$MUJOCO_ROOT" && -d "$MUJOCO_ROOT" ]] || return 1
  MUJOCO_ROOT="$(cd "$MUJOCO_ROOT" && pwd)"
  return 0
}

# simulate/CMakeLists.txt consumes mujoco/include, mujoco/simulate and
# mujoco/lib. A MuJoCo pip wheel has the first and third but no simulate/,
# which otherwise surfaces as a baffling glfw_adapter.cc link error.
check_mujoco_root() {
  local missing=()
  [[ -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ]] || missing+=("include/mujoco/mujoco.h")
  [[ -f "$MUJOCO_ROOT/simulate/simulate.cc" ]]    || missing+=("simulate/simulate.cc")
  compgen -G "$MUJOCO_ROOT/lib/libmujoco.so*" >/dev/null || missing+=("lib/libmujoco.so*")
  if (( ${#missing[@]} )); then
    die "'$MUJOCO_ROOT' is not a complete MuJoCo distribution.
           missing: ${missing[*]}
           simulate/CMakeLists.txt needs all three. A pip-installed mujoco
           package has no simulate/ sources; download a MuJoCo release from
           https://github.com/google-deepmind/mujoco/releases and pass
           --mujoco /path/to/mujoco-<version>." 30
  fi
}

resolve_model_src() {
  if [[ -n "$MODEL_SRC" ]]; then
    [[ -e "$MODEL_SRC" ]] || die "--model path does not exist: $MODEL_SRC" 30
    return 0
  fi
  if [[ -f "$REPO_A/robot_model/$MODEL_ZIP_NAME" ]]; then
    MODEL_SRC="$REPO_A/robot_model/$MODEL_ZIP_NAME"
    return 0
  fi
  # The assets may already be in place from an earlier run.
  if model_dir_complete; then
    MODEL_SRC=""
    return 0
  fi
  die "the go2-d1 model assets are missing and $REPO_A/robot_model/$MODEL_ZIP_NAME
           is not present. Pass --model /path/to/$MODEL_ZIP_NAME (or to an
           already-unpacked go2-d1 directory)." 30
}

model_dir_complete() {
  local d="$REPO_B/unitree_robots/$ROBOT_NAME"
  [[ -f "$d/go2-d1.xml" && -f "$d/scene.xml" && -d "$d/meshes/visual" && -d "$d/meshes/collision" ]]
}

# --------------------------------------------------------------- preflight ---

require_cmds() {
  local c missing=()
  for c in python3 git cmake unzip; do
    command -v "$c" >/dev/null 2>&1 || missing+=("$c")
  done
  if (( ${#missing[@]} )); then
    die "missing required tools: ${missing[*]}
           python3 is needed for the edit engine (multi-line insertion with an
           exact match-count assertion). unitree_mujoco already ships
           simulate_python/, so any machine that runs it has python3." 30
  fi
  python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)' \
    || die "python3 >= 3.8 required, found $(python3 -V 2>&1)" 30
}

check_repo_b_layout() {
  local f missing=()
  for f in "${TARGET_FILES[@]}" simulate/src/param.h; do
    [[ -f "$REPO_B/$f" ]] || missing+=("$f")
  done
  [[ -d "$REPO_B/unitree_robots" ]] || missing+=("unitree_robots/")
  if (( ${#missing[@]} )); then
    die "'$REPO_B' does not look like a unitree_mujoco checkout.
           missing: ${missing[*]}" 30
  fi
}

repo_b_head() {
  git -C "$REPO_B" rev-parse --short HEAD 2>/dev/null || echo ""
}

repo_b_is_git() {
  local top
  top="$(git -C "$REPO_B" rev-parse --show-toplevel 2>/dev/null)" || return 1
  [[ "$(cd "$top" && pwd)" == "$REPO_B" ]]
}

check_repo_b_head() {
  repo_b_is_git || { info "'$REPO_B' is not a git checkout; skipping revision check"; return 0; }
  local head; head="$(repo_b_head)"
  if [[ -n "$head" && "$head" != "$KNOWN_GOOD_HEAD"* ]]; then
    warn "unitree_mujoco is at $head; these anchors were verified against $KNOWN_GOOD_HEAD.
           That is fine -- the anchor check is the real gate. Only 'main.resolve_idl'
           is revision-sensitive, and it is optional."
  fi
}

check_arm_install() {
  local missing=()
  [[ -f "$ARM_CONFIG" ]] || missing+=("$ARM_CONFIG")
  [[ -f "$ARM_HEADER" ]] || missing+=("$ARM_HEADER")
  if (( ${#missing[@]} )); then
    die "unitree_arm is not fully installed under $ARM_PREFIX.
           missing:
$(printf '             %s\n' "${missing[@]}")
           find_package() needs the cmake config and the bridge #include needs
           the header, so both must be present. From $REPO_A:
             cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$ARM_PREFIX
             cmake --build build -j\$(nproc)
             sudo cmake --install build" 30
  fi
  # A stale install is the trap: the config is old enough to satisfy
  # find_package while the header predates the bridge.
  if ! grep -q "StartMujocoArmBridge" "$ARM_HEADER" 2>/dev/null; then
    die "$ARM_HEADER exists but has no StartMujocoArmBridge() -- the install is
           stale. Re-install unitree_arm from $REPO_A (see the commands above)." 30
  fi
}

check_writable() {
  local f unwritable=()
  for f in "${TARGET_FILES[@]}"; do
    [[ -w "$REPO_B/$f" ]] || unwritable+=("$f")
  done
  if (( DO_PROVISION )); then
    [[ -w "$REPO_B/unitree_robots" ]] || unwritable+=("unitree_robots/")
    [[ -w "$REPO_B/simulate" ]] || unwritable+=("simulate/")
  fi
  if (( ${#unwritable[@]} )); then
    die "not writable in '$REPO_B': ${unwritable[*]}" 30
  fi
  return 0
}

# Modified relative to git HEAD but carrying none of our payloads means the
# user hand-edited these files; overwriting would lose their work.
check_hand_edited() {
  repo_b_is_git || return 0
  local dirty
  dirty="$(git -C "$REPO_B" diff --name-only -- "${TARGET_FILES[@]}" 2>/dev/null || true)"
  [[ -z "$dirty" ]] && return 0
  local f marker suspicious=()
  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    # config.yaml has no C++ marker to look for; its evidence is the value we set.
    if [[ "$f" == "simulate/config.yaml" ]]; then
      marker="$ROBOT_NAME"
    else
      marker="d1_mujoco_bridge.h\|unitree_arm\|StartMujocoArmBridge\|MujocoArmLayout"
    fi
    grep -q "$marker" "$REPO_B/$f" 2>/dev/null || suspicious+=("$f")
  done <<< "$dirty"
  if (( ${#suspicious[@]} )); then
    warn "these files have local changes unrelated to the arm bridge:
$(printf '             %s\n' "${suspicious[@]}")
           The originals are backed up before anything is written, but review
           them first if the changes matter."
    confirm "Continue anyway?" || die "aborted by user" 1
  fi
}

confirm() {
  if (( ASSUME_YES || FORCE )); then return 0; fi
  [[ -t 0 ]] || return 0
  local reply
  printf '%s[d1-setup]%s %s [y/N] ' "$C_DIM" "$C_OFF" "$1" >&2
  read -r reply || true
  [[ "$reply" =~ ^[Yy] ]]
}

# ------------------------------------------------------------ edit engine ----
#
# The engine runs once per phase and speaks a tab-separated protocol on stdout:
#   STATE<TAB><hunk id><TAB><APPLIED|PENDING|ANCHOR_MISSING|ANCHOR_AMBIGUOUS|SKIPPED><TAB><detail>
#   RESULT<TAB>applied=N<TAB>pending=N<TAB>missing=N<TAB>ambiguous=N
# Human-readable diagnostics go to stderr; the exit code carries the verdict:
#   0 no blocking problem, 20 anchor missing/ambiguous, 11 partial state.
#
# The heredoc is quoted, so no shell expansion ever happens inside the python
# source; every path and option arrives through argv.

run_engine() {
  local mode="$1" stage="${2:-}"
  python3 - "$mode" "$REPO_B" "$stage" "$ROBOT_NAME" "$DOMAIN_ID" "$ALLOW_MISSING" <<'PY'
import os
import re
import sys

MODE, REPO_B, STAGE, ROBOT_NAME, DOMAIN_ID, ALLOW_MISSING = sys.argv[1:7]
ALLOWED = {x for x in ALLOW_MISSING.split(",") if x}

CFG = "simulate/config.yaml"
CML = "simulate/CMakeLists.txt"
BRG = "simulate/src/unitree_sdk2_bridge.h"
MAIN = "simulate/src/main.cc"

# ---------------------------------------------------------------------------
# The hunk table is the single source of truth.
#
#   op        insert_after | replace_line | set_scalar
#   anchor    one line that must appear EXACTLY once (leading whitespace is
#             significant, trailing whitespace is ignored)
#   payload   lines written literally, indentation baked in
#   sentinel  substring proving THIS hunk is already applied; it names the
#             hunk's semantic content, never its comment text, so reworded
#             comments cannot make an applied hunk look pending (which would
#             insert it a second time)
#   exact     match the sentinel as a whole line rather than a substring
#   tier      required   -> a missing anchor aborts the whole run
#             degradable -> a missing anchor warns and continues
#   since     upstream commit that introduced the anchor (for error messages)
#
# Detection uses the sentinel; post-apply validation uses the payload. The two
# answer different questions and must not be conflated.
# ---------------------------------------------------------------------------

HUNKS = [
    dict(
        id="yaml.robot", file=CFG, op="set_scalar", key="robot",
        value='"%s"' % ROBOT_NAME, tier="required", since=None,
        readme="Simulation (unitree_mujoco) > simulate/config.yaml",
        note="without this the simulator loads a model with no d1_J0 actuator, "
             "so the bridge starts and then does nothing",
    ),
    dict(
        id="yaml.domain_id", file=CFG, op="set_scalar", key="domain_id",
        value=DOMAIN_ID, tier="required", since=None,
        readme="Simulation (unitree_mujoco) > simulate/config.yaml",
        note="the SDK example programs hardcode DDS domain 0",
    ),
    dict(
        id="cmake.find_package", file=CML, op="insert_after",
        anchor="find_package(unitree_sdk2 REQUIRED)",
        payload=["find_package(unitree_arm REQUIRED)   # D1 arm (d1_sdk_extension)"],
        sentinel="find_package(unitree_arm", tier="required", since=None,
        readme="Simulation (unitree_mujoco) > simulate/CMakeLists.txt",
    ),
    dict(
        id="cmake.link_lib", file=CML, op="insert_after",
        # Whole-line match: the bare substring "unitree_sdk2" also occurs in
        # the find_package line above.
        anchor="  unitree_sdk2",
        payload=["  unitree_arm"],
        sentinel="  unitree_arm", exact=True, tier="required", since=None,
        readme="Simulation (unitree_mujoco) > simulate/CMakeLists.txt",
    ),
    dict(
        id="bridge.include", file=BRG, op="insert_after",
        anchor="#include <unitree/idl/hg/IMUState_.hpp>",
        payload=[
            "",
            "// [D1] Unitree D1-550 arm, from the separately installed unitree_arm",
            "// package. Everything the arm needs lives there; the edits marked [D1]",
            "// below are no-ops on a model without arm actuators.",
            "#include <unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h>",
        ],
        sentinel="unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h",
        tier="required", since="df74e71",
        readme="Simulation (unitree_mujoco) > simulate/src/unitree_sdk2_bridge.h",
    ),
    dict(
        id="bridge.member", file=BRG, op="insert_after",
        anchor="    int dim_motor_sensor_ = 0;",
        payload=[
            "",
            "    // [D1] Where the arm sits in ctrl; invalid when the model has no arm.",
            "    unitree::robot::d1::MujocoArmLayout arm_;",
        ],
        sentinel="MujocoArmLayout arm_;", tier="required", since="df74e71",
        readme="Simulation (unitree_mujoco) > simulate/src/unitree_sdk2_bridge.h",
    ),
    dict(
        id="bridge.detect", file=BRG, op="insert_after",
        anchor="        num_motor_ = mj_model_->nu;",
        payload=[
            "",
            "        // [D1] Hand the arm's actuators to the D1 protocol before",
            "        // num_motor_ is used anywhere. Detect() returns an invalid layout",
            "        // for a model without an arm, leaving num_motor_ at",
            "        // mj_model_->nu as it has always been.",
            "        arm_ = unitree::robot::d1::MujocoArmLayout::Detect(mj_model_);",
            "        if (arm_.valid()) num_motor_ = arm_.base;",
            "",
        ],
        sentinel="MujocoArmLayout::Detect(mj_model_)",
        tier="required", since="df74e71",
        readme="Simulation (unitree_mujoco) > simulate/src/unitree_sdk2_bridge.h",
    ),
    dict(
        id="main.start_bridge", file=MAIN, op="insert_after",
        # "while (true)" occurs twice in this file and must not be used.
        anchor="  interface->start();",
        payload=[
            "",
            "  // [D1] Serve the arm when the model has one; nullptr otherwise. Shares",
            "  // the ChannelFactory initialised above, so the arm answers on the",
            "  // robot's own DDS domain.",
            "  auto arm_bridge = unitree::robot::d1::StartMujocoArmBridge(m, d);",
        ],
        sentinel="StartMujocoArmBridge", tier="required", since=None,
        readme="Simulation (unitree_mujoco) > simulate/src/main.cc",
    ),
    dict(
        id="main.resolve_idl", file=MAIN, op="replace_line",
        anchor="    idl_type = param::ResolveIdlType(param::config.robot, m->nu, param::config.idl_type);",
        payload=[
            "    // [D1] Count only the actuators the Unitree protocol carries: the",
            "    // arm's servos travel on their own topic and must not push the model",
            "    // over the IDL's motor limit. Detect() yields an invalid layout",
            "    // without an arm.",
            "    const auto d1_layout = unitree::robot::d1::MujocoArmLayout::Detect(m);",
            "    const int unitree_motors = d1_layout.valid() ? d1_layout.base : m->nu;",
            "    idl_type = param::ResolveIdlType(param::config.robot, unitree_motors, param::config.idl_type);",
        ],
        sentinel="const int unitree_motors", tier="degradable", since="0244cc8",
        readme="Simulation (unitree_mujoco) > simulate/src/main.cc",
        note="this hunk is hardening, not a requirement: go2-d1 has 19 actuators "
             "and the unitree_go IDL allows 20, so IDL selection is already correct "
             "without it",
    ),
]

SCALAR_RE = {
    h["key"]: re.compile(r"^(?P<key>%s)(?P<pre>\s*):(?P<gap>\s*)(?P<val>\S+)(?P<tail>.*)$"
                         % re.escape(h["key"]))
    for h in HUNKS if h["op"] == "set_scalar"
}


def read_lines(rel):
    with open(os.path.join(REPO_B, rel), "r", encoding="utf-8", newline="") as fh:
        return fh.read().split("\n")


def find_anchor(lines, anchor):
    want = anchor.rstrip()
    return [i for i, ln in enumerate(lines) if ln.rstrip() == want]


def find_block(lines, payload):
    """Indices where the payload appears as a contiguous run."""
    want = [p.rstrip() for p in payload]
    n = len(want)
    if n == 0:
        return []
    have = [ln.rstrip() for ln in lines]
    return [i for i in range(len(have) - n + 1) if have[i:i + n] == want]


def find_scalar(lines, key):
    rx = SCALAR_RE[key]
    return [(i, m) for i, ln in enumerate(lines) for m in [rx.match(ln)] if m]


def find_sentinel(lines, h):
    """Indices of lines proving this hunk is already applied."""
    want = h["sentinel"]
    if h.get("exact"):
        want = want.rstrip()
        return [i for i, ln in enumerate(lines) if ln.rstrip() == want]
    return [i for i, ln in enumerate(lines) if want in ln]


def emit(kind, *fields):
    sys.stdout.write("\t".join([kind] + [str(f) for f in fields]) + "\n")


def report_anchor_problem(h, count, hits, lines):
    # An ambiguous anchor is always fatal: guessing which line to edit is worse
    # than stopping. A missing anchor is fatal only for a required hunk, and
    # --allow-missing is the expert escape hatch for those.
    if count > 1:
        fatal = True
    else:
        fatal = h["tier"] == "required" and h["id"] not in ALLOWED
    label = "FATAL" if fatal else "warn "
    out = [""]
    if count > 1:
        out.append("  %s anchor is ambiguous -- refusing to guess which one" % label)
    elif fatal:
        out.append("  %s anchor not found -- nothing was modified" % label)
    else:
        out.append("  %s anchor not found -- skipping this hunk" % label)
    out.append("")
    out.append("  hunk    %-24s tier=%s" % (h["id"], h["tier"]))
    out.append("  file    %s" % os.path.join(REPO_B, h["file"]))
    out.append("  op      %s" % h["op"])
    # A leading '|' keeps leading whitespace visible; two anchors are
    # indentation-sensitive.
    out.append("  anchor  |%s" % h.get("anchor", h.get("key", "")))
    out.append("  found   %d occurrence%s (expected exactly 1)"
               % (count, "" if count == 1 else "s"))
    if hits:
        out.append("  lines   %s" % ", ".join(str(i + 1) for i in hits))
    out.append("")
    if h.get("since"):
        out.append("  This anchor arrived upstream in %s; older checkouts do not have it."
                   % h["since"])
    if h.get("note"):
        out.append("  %s." % h["note"][0].upper() + h["note"][1:])
    out.append("")
    if not fatal:
        out.append("    continuing without it; the rest of the wiring is unaffected.")
    elif h["tier"] == "degradable":
        out.append("    skip it   ./setup.sh install --allow-missing %s" % h["id"])
    out.append("    by hand   README.md  >  %s" % h["readme"])
    out.append("")
    sys.stderr.write("\n".join(out) + "\n")
    return fatal


def plan():
    """Resolve every hunk. Returns (states, staged, problems)."""
    by_file = {}
    for h in HUNKS:
        by_file.setdefault(h["file"], []).append(h)

    states, staged, fatal_any = {}, {}, False

    for rel, hunks in by_file.items():
        lines = read_lines(rel)
        original_len = len(lines)
        expected_growth = 0

        # Resolve top-down so that later indices stay valid; apply bottom-up.
        pending = []
        for h in hunks:
            if h["op"] == "set_scalar":
                hits = find_scalar(lines, h["key"])
                if len(hits) != 1:
                    states[h["id"]] = ("ANCHOR_MISSING" if not hits else "ANCHOR_AMBIGUOUS",
                                       "expected 1, found %d" % len(hits))
                    if report_anchor_problem(h, len(hits), [i for i, _ in hits], lines):
                        fatal_any = True
                    continue
                idx, m = hits[0]
                if m.group("val") == h["value"]:
                    states[h["id"]] = ("APPLIED", "%s already %s" % (h["key"], h["value"]))
                    continue
                states[h["id"]] = ("PENDING", "line %d" % (idx + 1))
                pending.append((idx, h, m))
                continue

            marks = find_sentinel(lines, h)
            if len(marks) == 1:
                states[h["id"]] = ("APPLIED", "sentinel at line %d" % (marks[0] + 1))
                continue
            if len(marks) > 1:
                states[h["id"]] = ("ANCHOR_AMBIGUOUS",
                                   "sentinel present %d times" % len(marks))
                sys.stderr.write(
                    "\n  FATAL hunk %s: '%s' appears %d times in %s (lines %s) -- the\n"
                    "        file looks doubly patched. Revert it before installing.\n\n"
                    % (h["id"], h["sentinel"], len(marks), h["file"],
                       ", ".join(str(i + 1) for i in marks)))
                fatal_any = True
                continue

            hits = find_anchor(lines, h["anchor"])
            if len(hits) != 1:
                states[h["id"]] = ("ANCHOR_MISSING" if not hits else "ANCHOR_AMBIGUOUS",
                                   "expected 1, found %d" % len(hits))
                if report_anchor_problem(h, len(hits), hits, lines):
                    fatal_any = True
                else:
                    states[h["id"]] = ("SKIPPED", "anchor absent, allowed by --allow-missing")
                continue

            states[h["id"]] = ("PENDING", "anchor at line %d" % (hits[0] + 1))
            pending.append((hits[0], h, None))
            expected_growth += len(h["payload"]) - (1 if h["op"] == "replace_line" else 0)

        if not pending:
            continue

        for idx, h, m in sorted(pending, key=lambda t: -t[0]):
            if h["op"] == "insert_after":
                lines[idx + 1:idx + 1] = list(h["payload"])
            elif h["op"] == "replace_line":
                lines[idx:idx + 1] = list(h["payload"])
            elif h["op"] == "set_scalar":
                # Rewrite only the value span; carry the inline comment
                # verbatim, because upstream keeps changing it.
                lines[idx] = "%s%s:%s%s%s" % (m.group("key"), m.group("pre"),
                                              m.group("gap"), h["value"], m.group("tail"))
        staged[rel] = (lines, original_len, expected_growth)

    return states, staged, fatal_any


def validate(staged, states):
    """Post-conditions checked independently of the code that produced them."""
    errs = []
    for rel, (lines, original_len, expected_growth) in staged.items():
        if len(lines) != original_len + expected_growth:
            errs.append("%s: line count %d, expected %d"
                        % (rel, len(lines), original_len + expected_growth))
        for h in HUNKS:
            if h["file"] != rel or h["op"] == "set_scalar":
                continue
            # Only what this run wrote: an already-applied hunk may carry an
            # older wording of the same payload, which is fine and not ours to
            # rewrite.
            if states.get(h["id"], ("", ""))[0] != "PENDING":
                continue
            n = len(find_block(lines, h["payload"]))
            if n != 1:
                errs.append("%s: payload of %s present %d times, expected 1"
                            % (rel, h["id"], n))

    if CML in staged:
        lines = staged[CML][0]
        text = "\n".join(lines)
        if text.count("(") != text.count(")"):
            errs.append("%s: unbalanced parentheses" % CML)
        # unitree_arm must sit inside the link_libraries() block.
        start = next((i for i, ln in enumerate(lines)
                      if ln.strip().startswith("link_libraries(")), None)
        if start is None:
            errs.append("%s: no link_libraries( block" % CML)
        else:
            end = next((i for i in range(start, len(lines))
                        if lines[i].strip() == ")"), None)
            arm = next((i for i, ln in enumerate(lines) if ln.rstrip() == "  unitree_arm"), None)
            if end is None or arm is None or not (start < arm < end):
                errs.append("%s: 'unitree_arm' is not inside link_libraries()" % CML)

    if CFG in staged:
        lines = staged[CFG][0]
        before = read_lines(CFG)
        keys = lambda ls: sorted(ln.split(":", 1)[0] for ln in ls
                                 if ln[:1].isalpha() and ":" in ln)
        if keys(lines) != keys(before):
            errs.append("%s: top-level key set changed" % CFG)
        if len(lines) != len(before):
            errs.append("%s: line count changed" % CFG)

    return errs


def main():
    states, staged, fatal_any = plan()

    for h in HUNKS:
        state, detail = states.get(h["id"], ("UNKNOWN", ""))
        emit("STATE", h["id"], state, detail)

    counts = {"applied": 0, "pending": 0, "missing": 0, "ambiguous": 0, "skipped": 0}
    for state, _ in states.values():
        key = {"APPLIED": "applied", "PENDING": "pending", "ANCHOR_MISSING": "missing",
               "ANCHOR_AMBIGUOUS": "ambiguous", "SKIPPED": "skipped"}.get(state)
        if key:
            counts[key] += 1
    emit("RESULT", *["%s=%d" % (k, v) for k, v in counts.items()])

    if fatal_any:
        sys.stderr.write("  All target files are unchanged. Re-run when resolved.\n\n")
        return 20

    if MODE == "state":
        # Mixed applied/pending is a partial install and must be refused.
        if counts["applied"] and counts["pending"]:
            return 11
        return 0

    errs = validate(staged, states)
    if errs:
        sys.stderr.write("\n  FATAL staged output failed validation:\n")
        for e in errs:
            sys.stderr.write("    - %s\n" % e)
        sys.stderr.write("  Nothing was written.\n\n")
        return 20

    # Staging writes into STAGE, never into the target tree, so it is safe to
    # stage a mixed result and let the caller decide whether to commit it.
    for rel, (lines, _, _) in staged.items():
        dst = os.path.join(STAGE, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "w", encoding="utf-8", newline="") as fh:
            fh.write("\n".join(lines))

    if counts["applied"] and counts["pending"]:
        return 11

    return 0


sys.exit(main())
PY
}

# Parse the engine's STATE/RESULT stream into globals.
declare -A HUNK_STATE=()
ENGINE_RESULT=""

run_engine_capture() {
  local mode="$1" stage="${2:-}" out rc=0
  HUNK_STATE=(); ENGINE_RESULT=""
  out="$(run_engine "$mode" "$stage")" || rc=$?
  local line kind id state detail
  while IFS=$'\t' read -r kind id state detail; do
    case "$kind" in
      STATE)  HUNK_STATE["$id"]="$state"$'\t'"$detail" ;;
      RESULT) ENGINE_RESULT="$id $state $detail" ;;
    esac
  done <<< "$out"
  return "$rc"
}

print_hunk_table() {
  local id row state detail colour
  printf '\n  %-22s %-18s %s\n' "HUNK" "STATE" "DETAIL" >&2
  printf '  %-22s %-18s %s\n' "----" "-----" "------" >&2
  for id in yaml.robot yaml.domain_id cmake.find_package cmake.link_lib \
            bridge.include bridge.member bridge.detect \
            main.start_bridge main.resolve_idl; do
    row="${HUNK_STATE[$id]:-UNKNOWN$'\t'}"
    state="${row%%$'\t'*}"; detail="${row#*$'\t'}"
    case "$state" in
      APPLIED) colour="$C_GRN" ;;
      PENDING) colour="$C_DIM" ;;
      SKIPPED) colour="$C_YEL" ;;
      *)       colour="$C_RED" ;;
    esac
    printf '  %-22s %s%-18s%s %s\n' "$id" "$colour" "$state" "$C_OFF" "$detail" >&2
  done
  printf '\n' >&2
}

# --------------------------------------------------------- backup / commit ---

make_backup() {
  local slug; slug="$(printf '%s' "$REPO_B" | tr '/' '_' | sed 's/^_//')"
  BACKUP_DIR="$BACKUP_ROOT/$slug/$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$BACKUP_DIR"
  local f
  for f in "${TARGET_FILES[@]}"; do
    mkdir -p "$BACKUP_DIR/$(dirname "$f")"
    cp -p "$REPO_B/$f" "$BACKUP_DIR/$f"
  done
  write_manifest "in_progress"
  dbg "backup: $BACKUP_DIR"
}

write_manifest() {
  local state="$1" f
  {
    printf '{\n'
    printf '  "state": "%s",\n' "$state"
    printf '  "created": "%s",\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf '  "script_version": "%s",\n' "$SCRIPT_VERSION"
    printf '  "repo_a": "%s",\n' "$REPO_A"
    printf '  "repo_b": "%s",\n' "$REPO_B"
    printf '  "repo_b_head": "%s",\n' "$(repo_b_head)"
    printf '  "robot_name": "%s",\n' "$ROBOT_NAME"
    printf '  "domain_id": "%s",\n' "$DOMAIN_ID"
    printf '  "files": [\n'
    local first=1
    for f in "${TARGET_FILES[@]}"; do
      (( first )) || printf ',\n'
      first=0
      printf '    { "path": "%s", "mode": "%s", "sha256": "%s" }' \
        "$f" "$(stat -c '%a' "$BACKUP_DIR/$f")" "$(sha256sum "$BACKUP_DIR/$f" | cut -d' ' -f1)"
    done
    printf '\n  ]\n}\n'
  } > "$BACKUP_DIR/manifest.json"
}

# cp onto an existing file preserves the destination's mode and inode, which is
# what keeps simulate/src/main.cc at 755; mv would drop it to 644.
commit_staged() {
  local f
  for f in "${TARGET_FILES[@]}"; do
    [[ -f "$STAGE/$f" ]] || continue
    cp "$STAGE/$f" "$REPO_B/$f"
    dbg "wrote $f"
  done
  write_manifest "applied"
}

show_dry_run_diff() {
  local f
  for f in "${TARGET_FILES[@]}"; do
    [[ -f "$STAGE/$f" ]] || continue
    printf '\n--- a/%s\n+++ b/%s\n' "$f" "$f" >&2
    diff -U2 "$REPO_B/$f" "$STAGE/$f" | tail -n +3 >&2 || true
  done
  printf '\n' >&2
}

# ---------------------------------------------------------- provisioning -----

provision_symlink() {
  local link="$REPO_B/simulate/mujoco"
  if [[ -L "$link" ]]; then
    local target; target="$(readlink -f "$link" || true)"
    if [[ -n "$target" && -f "$target/include/mujoco/mujoco.h" ]]; then
      ok "simulate/mujoco -> $target (already valid)"
      return 0
    fi
    warn "simulate/mujoco is a broken symlink; replacing it"
    (( DRY_RUN )) || rm -f "$link"
  elif [[ -d "$link" ]]; then
    warn "simulate/mujoco is a real directory (vendored MuJoCo?); leaving it alone"
    return 0
  fi
  resolve_mujoco_root || die "no MuJoCo distribution found. Pass --mujoco /path/to/mujoco-<version>." 30
  check_mujoco_root
  if (( DRY_RUN )); then
    info "would link simulate/mujoco -> $MUJOCO_ROOT"
    return 0
  fi
  # Never 'ln -sf' onto an existing directory: it nests the link inside it.
  ln -s "$MUJOCO_ROOT" "$link"
  ok "simulate/mujoco -> $MUJOCO_ROOT"
}

provision_model() {
  local dst="$REPO_B/unitree_robots/$ROBOT_NAME"
  if model_dir_complete; then
    ok "unitree_robots/$ROBOT_NAME (already present)"
    assert_arm_actuator
    return 0
  fi
  if [[ -e "$dst" ]]; then
    warn "unitree_robots/$ROBOT_NAME exists but is incomplete"
    confirm "Replace it?" || die "aborted by user; a truncated model gives an opaque MuJoCo XML load error" 1
    (( DRY_RUN )) || rm -rf "$dst"
  fi
  resolve_model_src
  [[ -n "$MODEL_SRC" ]] || return 0

  if (( DRY_RUN )); then
    info "would install unitree_robots/$ROBOT_NAME from $MODEL_SRC"
    return 0
  fi

  local scratch="$STAGE/model"
  mkdir -p "$scratch"
  if [[ -d "$MODEL_SRC" ]]; then
    cp -r "$MODEL_SRC" "$scratch/$MODEL_ZIP_ROOT"
  else
    unzip -q "$MODEL_SRC" -d "$scratch"
    [[ -d "$scratch/$MODEL_ZIP_ROOT" ]] \
      || die "'$MODEL_SRC' does not unpack to a '$MODEL_ZIP_ROOT/' directory" 30
  fi
  # The directory name must equal config.yaml's robot value: main.cc resolves
  # <repo>/unitree_robots/<robot>/<robot_scene> at runtime.
  mv "$scratch/$MODEL_ZIP_ROOT" "$dst"
  ok "unitree_robots/$ROBOT_NAME installed from $(basename "$MODEL_SRC")"
  assert_arm_actuator
  info "note: unitree_robots/ is not gitignored, so this shows as untracked in $REPO_B"
}

# MujocoArmLayout::Detect keys off an actuator literally named d1_J0. Without
# it the bridge starts and silently does nothing, so check here rather than
# leaving it to be discovered at runtime.
assert_arm_actuator() {
  local xml="$REPO_B/unitree_robots/$ROBOT_NAME/go2-d1.xml"
  [[ -f "$xml" ]] || return 0
  if ! grep -q "$ARM_ACTUATOR" "$xml"; then
    die "$xml has no actuator $ARM_ACTUATOR.
           MujocoArmLayout::Detect() keys off that exact name, so the bridge
           would start and then never drive anything." 30
  fi
  dbg "model exposes actuator d1_J0"
}

# ------------------------------------------------------------ subcommands ----

preflight() {
  step "preflight"
  require_cmds
  resolve_repo_b
  check_repo_b_layout
  check_repo_b_head
  if (( DO_EDITS )); then check_arm_install; fi
  check_writable
  if (( DO_EDITS )); then check_hand_edited; fi
  local head; head="$(repo_b_head)"
  ok "unitree_mujoco: $REPO_B${head:+ (@$head)}"
  return 0
}

cmd_check() {
  preflight
  STAGE="$(mktemp -d)"
  local rc=0
  run_engine_capture state || rc=$?
  print_hunk_table
  case "$rc" in
    0)
      if [[ "$ENGINE_RESULT" == *"pending=0"* ]]; then
        ok "already fully wired"; exit 0
      fi
      info "not wired yet -- run: ./setup.sh install"; exit 10 ;;
    11)
      warn "PARTIALLY wired. Re-running install would insert on top of edits that
           are already there. Undo first:
             ./setup.sh revert
             ./setup.sh install
           or force a resume of only the pending hunks:
             ./setup.sh install --force"
      exit 11 ;;
    *) exit "$rc" ;;
  esac
}

cmd_status() {
  resolve_repo_b
  check_repo_b_layout
  STAGE="$(mktemp -d)"
  local rc=0
  run_engine_capture state || rc=$?
  print_hunk_table
  printf '  repo   %s\n' "$REPO_B" >&2
  printf '  head   %s (anchors verified against %s)\n' "$(repo_b_head)" "$KNOWN_GOOD_HEAD" >&2
  printf '  model  %s\n' "$(model_dir_complete && echo present || echo MISSING)" >&2
  printf '  mujoco %s\n' "$([[ -e "$REPO_B/simulate/mujoco" ]] && echo present || echo MISSING)" >&2
  printf '\n' >&2
  exit "$rc"
}

cmd_install() {
  preflight
  STAGE="$(mktemp -d)"

  if (( DO_EDITS )); then
    step "planning edits"
    local rc=0
    run_engine_capture stage "$STAGE" || rc=$?

    if (( rc == 11 )) && (( ! FORCE )); then
      print_hunk_table
      die "PARTIALLY wired -- refusing to patch on top of it.
           Undo and start clean:   ./setup.sh revert && ./setup.sh install
           Or resume the pending hunks only:  ./setup.sh install --force" 11
    fi
    if (( rc == 20 )); then exit 20; fi
    if (( rc != 0 && rc != 11 )); then die "edit engine failed (exit $rc)" "$rc"; fi

    if (( VERBOSE )); then print_hunk_table; fi

    if [[ "$ENGINE_RESULT" == *"pending=0"* ]]; then
      ok "edits already applied; nothing to write"
    elif (( DRY_RUN )); then
      step "dry run -- these changes would be written"
      show_dry_run_diff
    else
      make_backup
      commit_staged
      ok "edits applied (backup: $BACKUP_DIR)"
    fi
  fi

  if (( DO_PROVISION )); then
    step "provisioning assets"
    provision_symlink
    provision_model
  fi

  if (( DRY_RUN )); then
    info "dry run complete; nothing was written"
    return 0
  fi

  step "next"
  cat >&2 <<EOF
  Build and confirm the bridge actually starts:

    ./setup.sh verify

  or by hand:

    cmake -S $REPO_B/simulate -B $REPO_B/simulate/build
    cmake --build $REPO_B/simulate/build -j\$(nproc)
    cd $REPO_B/simulate/build && ./unitree_mujoco -r $ROBOT_NAME

  The simulator must print, at startup:

    [d1] serving the arm on rt/arm_Command: ctrl[12..18] at 10 Hz

  If that line is absent the bridge is not running, whatever the build said.
EOF
}

cmd_revert() {
  resolve_repo_b
  check_repo_b_layout
  local slug; slug="$(printf '%s' "$REPO_B" | tr '/' '_' | sed 's/^_//')"
  local newest
  newest="$(find "$BACKUP_ROOT/$slug" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | sort | tail -1)"
  [[ -n "$newest" ]] || die "no backup found under $BACKUP_ROOT/$slug
           Nothing to revert from. If this repo is a git checkout you can use
             git -C $REPO_B checkout -- simulate/
           but note that discards any other local changes too." 1

  info "restoring from $newest"
  local f changed=0
  for f in "${TARGET_FILES[@]}"; do
    [[ -f "$newest/$f" ]] || continue
    if cmp -s "$newest/$f" "$REPO_B/$f"; then
      dbg "$f already matches the backup"
      continue
    fi
    if (( DRY_RUN )); then
      info "would restore $f"
    else
      cp "$newest/$f" "$REPO_B/$f"
      dbg "restored $f"
    fi
    changed=1
  done
  (( changed )) || { ok "the four files already match the backup"; return 0; }
  if (( DRY_RUN )); then info "dry run complete"; return 0; fi
  ok "restored the four files"
  info "assets were left in place; remove them yourself if you want to:
             rm -rf $REPO_B/unitree_robots/$ROBOT_NAME
             rm -f  $REPO_B/simulate/mujoco"
}

cmd_verify() {
  resolve_repo_b
  check_repo_b_layout
  STAGE="$(mktemp -d)"

  step "1/4 static state"
  local rc=0
  run_engine_capture state || rc=$?
  if (( rc != 0 )) || [[ "$ENGINE_RESULT" != *"pending=0"* ]]; then
    print_hunk_table
    die "the wiring is not fully applied; run ./setup.sh install first" 40
  fi
  ok "all hunks applied"

  step "2/4 assets"
  local link="$REPO_B/simulate/mujoco"
  [[ -f "$link/include/mujoco/mujoco.h" ]] || die "simulate/mujoco/include/mujoco/mujoco.h missing" 40
  [[ -f "$link/simulate/simulate.cc" ]]    || die "simulate/mujoco/simulate/simulate.cc missing" 40
  compgen -G "$link/lib/libmujoco.so*" >/dev/null || die "simulate/mujoco/lib/libmujoco.so* missing" 40
  model_dir_complete || die "unitree_robots/$ROBOT_NAME is missing or incomplete" 40
  assert_arm_actuator
  ok "MuJoCo and model assets in place"

  step "3/4 build"
  if ! cmake -S "$REPO_B/simulate" -B "$REPO_B/simulate/build" >"$STAGE/cmake.log" 2>&1; then
    tail -20 "$STAGE/cmake.log" >&2
    die "cmake configure failed. A 'unitree_arm' error here means the install
           under $ARM_PREFIX is stale." 40
  fi
  if ! cmake --build "$REPO_B/simulate/build" -j"$(nproc)" >"$STAGE/build.log" 2>&1; then
    tail -30 "$STAGE/build.log" >&2
    if grep -q "MujocoArmLayout" "$STAGE/build.log"; then
      die "build failed on MujocoArmLayout in main.cc -- the bridge.include hunk
           did not land, so the type is undeclared there." 40
    fi
    die "build failed; see the tail above" 40
  fi
  ok "simulator built"

  if (( ! RUN_SIM )); then
    info "--no-run: stopping before launching the simulator"
    return 0
  fi
  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    warn "no DISPLAY or WAYLAND_DISPLAY; skipping the runtime check.
           Run './setup.sh verify' from a graphical session to complete it."
    return 0
  fi

  step "4/4 runtime"
  local log="$STAGE/sim.log" pid banner=""
  ( cd "$REPO_B/simulate/build" && ./unitree_mujoco -r "$ROBOT_NAME" >"$log" 2>&1 ) &
  pid=$!
  local i
  for ((i = 0; i < 40; i++)); do
    if grep -q "serving the arm on" "$log" 2>/dev/null; then
      banner="$(grep -m1 "serving the arm on" "$log")"
      break
    fi
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.5
  done
  kill -TERM "$pid" 2>/dev/null || true
  ( sleep 3; kill -KILL "$pid" 2>/dev/null || true ) &
  wait "$pid" 2>/dev/null || true

  if [[ -z "$banner" ]]; then
    tail -20 "$log" >&2
    die "the simulator never printed the [d1] banner, so the bridge is not
           serving rt/arm_Command." 40
  fi
  info "$banner"
  # Match the whole range, not just the prefix: it proves Detect() found all
  # seven servos at the right base rather than merely running.
  if [[ "$banner" =~ ctrl\[([0-9]+)\.\.([0-9]+)\] ]]; then
    local lo="${BASH_REMATCH[1]}" hi="${BASH_REMATCH[2]}"
    if [[ "$lo" != "12" || "$hi" != "18" ]]; then
      die "expected ctrl[12..18] (12 leg actuators, then d1_J0..d1_J6) but got
           ctrl[$lo..$hi]; the model or the layout detection is off." 40
    fi
  else
    die "could not parse the ctrl range out of: $banner" 40
  fi
  ok "bridge serving ctrl[12..18] at 10 Hz"

  if (( FULL_VERIFY )); then
    step "extra  DDS round trip"
    local ex="$REPO_A/build/examples/get_arm_joint_angle"
    [[ -x "$ex" ]] || die "$ex not built; run: cmake --build $REPO_A/build -j" 40
    info "run this against a live simulator to exercise the whole path:
             $ex lo"
  fi
}

# ------------------------------------------------------------------ main -----

main() {
  parse_args "$@"
  case "$SUBCOMMAND" in
    help)    cmd_help ;;
    check)   cmd_check ;;
    status)  cmd_status ;;
    install) cmd_install ;;
    revert)  cmd_revert ;;
    verify)  cmd_verify ;;
  esac
}

main "$@"
