#!/bin/bash

set -euo pipefail

dir="$(dirname "$0")"

mode=""
core="SSMPFLescape"
use_gui=false
use_robot=false
use_traj=false
n_test=""
n_traj=""
Qpj=1.0
Qpt=1.0
Qv=1.0

# The robot trajectory executables, one per control strategy
valid_cores="SSM SSMPFL SSMPFLescape"

print_usage() {
    cat <<EOF
Usage: $(basename "$0") MODE [OPTIONS]

Modes (choose exactly one):
  --track           Start tracking procedure
  --record          Start recording procedure
  --stream          Start streaming procedure
  --calibrate       Start calibration procedure

Options:
  --gui             Launch the web interface
  --test N          Test number to stream (only used with --stream)
  --robot           Include robot simulation
  --traj N          Traj number to execute (only used with --robot)
  --core CORE       Robot control strategy: $(echo $valid_cores | tr ' ' '|')
                    (default: $core)
  -h, --help        Show this help message
EOF
}

# --- Parse arguments (order-independent) ---
while [ $# -gt 0 ]; do
    case "$1" in
        --track|--record|--stream|--calibrate|--x)
            if [ -n "$mode" ]; then
                echo "Error: only one mode may be specified (got '$mode' and '$1')." >&2
                exit 1
            fi
            mode="$1"
            shift
            ;;
        --gui)
            use_gui=true
            shift
            ;;
        --robot)
            use_robot=true
            shift
            ;;
        --test)
            n_test="${2:-}"
            if [ -z "$n_test" ]; then
                echo "Error: --test requires a value." >&2
                exit 1
            fi
            shift 2
            ;;
        --test=*)
            n_test="${1#*=}"
            shift
            ;;
        --traj)
            n_traj="${2:-}"
            if [ -z "$n_traj" ]; then
                echo "Error: --traj requires a value." >&2
                exit 1
            fi
            use_traj=true
            shift 2
            ;;
        --traj=*)
            n_traj="${1#*=}"
            use_traj=true
            shift
            ;;
        --core)
            core="${2:-}"
            if [ -z "$core" ]; then
                echo "Error: --core requires a value." >&2
                exit 1
            fi
            shift 2
            ;;
        --core=*)
            core="${1#*=}"
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "Error: unknown argument '$1'" >&2
            print_usage
            exit 1
            ;;
    esac
done


if [ -z "$mode" ]; then
    echo "Error: no mode specified." >&2
    print_usage
    exit 1
fi

# ── Validate the robot control strategy and the trajectory it needs ─────────
case " $valid_cores " in
    *" $core "*) ;;
    *)
        echo "Error: unknown core '$core'. Valid cores: $valid_cores" >&2
        exit 1
        ;;
esac

if [ "$use_robot" = true ] && [ "$use_traj" != true ]; then
    echo "Error: --robot requires a trajectory, add --traj N." >&2
    exit 1
fi

n_devices=$(lsusb | grep -c 8086 || true)

# ── Stop leftovers of a previous run: they keep the ZMQ ports (6000+) and the
# ── shared-memory segments busy, which makes every new run fail ──────────────
# (anchored so that e.g. a "gdb --args ./build/main_SSMPFL" session is not killed)
pkill -9 -f "^python3 $dir/scripts/(data_recording|data_merging|camera_stream|web_interface|calibration)\.py" 2>/dev/null || true
pkill -9 -f "^$dir/build/(rula_evaluation|main_)" 2>/dev/null || true
sleep 0.2
rm -f /dev/shm/shared_image* 2>/dev/null || true

trap 'kill 0' INT TERM

camera_stream="python3 $dir/scripts/camera_stream.py"
data_recording="python3 $dir/scripts/data_recording.py"
data_merging="python3 $dir/scripts/data_merging.py"
web_interface="python3 $dir/scripts/web_interface.py"
calibration="python3 $dir/scripts/calibration.py"
rula_evaluation="$dir/build/rula_evaluation"
exec_trajectory="$dir/build/main_$core"

# ── Start the robot trajectory executor, when the robot mode is requested ────
# The executor takes the trajectory index and the repository root, loads the
# trajectory from src/trajectories/test<N>/, receives the merged skeleton from
# data_merging.py and publishes the computed trajectory to web_interface.py.
start_robot() {
    if [ "$use_robot" != true ]; then
        return 0
    fi
    if [ ! -x "$exec_trajectory" ]; then
        echo "Error: '$exec_trajectory' not found. Build it first:" >&2
        echo "  mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j\$(nproc)" >&2
        exit 1
    fi
    echo "Starting the robot trajectory executor: $exec_trajectory (trajectory $n_traj)"
    $exec_trajectory "$n_traj" "$dir" &
}

case "$mode" in
    --track)
        echo "Starting tracking procedure..."
        $data_merging "$n_devices" &
        sleep 0.1
        $camera_stream &
        $rula_evaluation &
        start_robot
        if [ "$use_gui" = true ]; then
            sleep 1
            if [ "$use_robot" = true ]; then
                $web_interface "$n_devices" "--robot"
            else
                $web_interface "$n_devices"
            fi
        else
            wait
        fi
        ;;

    --record)
        echo "Starting recording procedure..."
        $data_recording "$n_devices" "-r" &
        sleep 0.1
        $camera_stream &
        wait
        ;;

    --stream)
        if [ -z "$n_test" ]; then
            echo -n "Enter the value of the test to stream: "
            read -r n_test
        fi

        n_devices=$(ls "$dir"/scripts/data/skeleton_data/test"$n_test"/skeleton* 2>/dev/null | wc -l)
        echo "Found $n_devices skeleton data for test $n_test."
        if [ "$n_devices" -eq 0 ]; then
            echo "Error: no skeleton data found for test $n_test." >&2
            exit 1
        fi

        echo "Starting streaming procedure..."
        $data_recording "$n_devices" "-s" "$n_test" &
        $data_merging "$n_devices" &
        $rula_evaluation &
        start_robot
        if [ "$use_gui" = true ]; then
            sleep 1
            if [ "$use_robot" = true ]; then
                $web_interface "$n_devices" "--robot"
            else
                $web_interface "$n_devices"
            fi
        else
            wait
        fi
        ;;

    --x)
        n_devices=$(ls "$dir"/scripts/data/skeleton_data/test7/skeleton* 2>/dev/null | wc -l)

        echo "Testing temp..."
        $data_recording "$n_devices" "-s" "10" &
        $data_merging "$n_devices" &
        $rula_evaluation &
        $exec_trajectory "3" "$dir" &
        if [ "$use_gui" = true ]; then
            $web_interface "$n_devices" "--robot"
        else
            wait
        fi
        ;;

    --calibrate)
        echo "Starting calibration procedure..."
        $calibration
        ;;
esac