#!/bin/bash
#
# Usage:
#   ./run.sh --track [--gui]
#   ./run.sh --record
#   ./run.sh --stream [--test N] [--gui]
#   ./run.sh --calibrate

set -euo pipefail

dir="$(dirname "$0")"

mode=""
use_gui=false
use_robot=false
use_traj=false
n_test=""
n_traj=""
Qpj=1.0
Qpt=1.0
Qv=1.0

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
  -h, --help        Show this help message
EOF
}

# --- Parse arguments (order-independent) ---
while [ $# -gt 0 ]; do
    case "$1" in
        --track|--record|--stream|--calibrate)
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

n_devices=$(lsusb | grep -c 8086 || true)

trap 'kill 0' INT

camera_stream="python3 $dir/scripts/camera_stream.py"
data_recording="python3 $dir/scripts/data_recording.py"
data_merging="python3 $dir/scripts/data_merging.py"
web_interface="python3 $dir/scripts/web_interface.py"
calibration="python3 $dir/scripts/calibration.py"
rula_evaluation="$dir/build/rula_evaluation"
exec_trajectory="$dir/build/main"

case "$mode" in
    --track)
        echo "Starting tracking procedure..."
        $data_merging "$n_devices" &
        sleep 0.1
        $camera_stream &
        $rula_evaluation &
        if [ "$use_robot" = true ]; then
            if [ "$use_traj" = true ]; then
                $exec_trajectory "$n_traj" "$dir" &
            fi
        fi
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
        if [ "$use_robot" = true ]; then
            if [ "$use_traj" = true ]; then
                $exec_trajectory "$n_traj" "$dir" &
            fi
        fi
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

    --calibrate)
        echo "Starting calibration procedure..."
        $calibration
        ;;
esac