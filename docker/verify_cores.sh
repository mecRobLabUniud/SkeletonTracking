#!/usr/bin/env bash
#
# Smoke-test all three robot control cores inside the container.
#
# Usage (from the repository root, inside the container):
#   ./docker/verify_cores.sh
#
# Environment overrides:
#   TEST=7 TRAJ=3 DURATION=90 LOG_DIR=/tmp/skeleton-tracking-logs
#
# For each core it starts the normal pipeline
#   ./run.sh --stream --test 7 --robot --traj 3 --core <core> --gui
# checks that the replay/merging/robot/web processes came up, requests the web
# page over HTTP, and then shuts the run down before moving to the next core.

set -uo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"

cores=(SSM SSMPFL SSMPFLescape)
test_num=${TEST:-7}
traj_num=${TRAJ:-3}
duration=${DURATION:-90}
log_dir=${LOG_DIR:-/tmp/skeleton-tracking-logs}
mkdir -p "$log_dir"

status=0

for core in "${cores[@]}"; do
    log="$log_dir/run-$core.log"
    echo "==================================================================="
    echo "Core: $core  (test $test_num, trajectory $traj_num)"
    echo "Log:  $log"
    rm -f "$log"

    timeout --signal=TERM --kill-after=10s "${duration}s" \
        ./run.sh --stream --test "$test_num" --robot --traj "$traj_num" \
                 --core "$core" --gui >"$log" 2>&1 &
    runner=$!

    web_ok=false
    robot_ok=false

    # Wait until the web interface and the robot executor are both up (or the
    # run dies / times out).
    while kill -0 "$runner" 2>/dev/null; do
        if [ "$web_ok" = false ] && grep -qE 'Running on (http|all addresses)' "$log"; then
            for _ in $(seq 1 20); do
                if curl -fsS --max-time 2 http://127.0.0.1:5000/ >/dev/null 2>&1; then
                    web_ok=true
                    echo "  web interface: OK"
                    break
                fi
                sleep 0.5
            done
            if [ "$web_ok" = false ]; then
                echo "  web interface: HTTP request failed"
            fi
        fi

        if [ "$robot_ok" = false ] && grep -q '\[STATE\] running' "$log"; then
            robot_ok=true
            echo "  robot executor: OK"
        fi

        if [ "$web_ok" = true ] && [ "$robot_ok" = true ]; then
            # Let the playback/collision loop run for a few more seconds.
            sleep 5
            break
        fi
        sleep 1
    done

    if kill -0 "$runner" 2>/dev/null; then
        kill -TERM "$runner" 2>/dev/null || true
    fi
    wait "$runner" 2>/dev/null || true

    # Do not let a leftover process hold the fixed ZMQ ports for the next core.
    pkill -9 -f "^python3 ./scripts/(data_recording|data_merging|camera_stream|web_interface|calibration)\.py" 2>/dev/null || true
    pkill -9 -f "^./build/(rula_evaluation|main_)" 2>/dev/null || true
    sleep 0.5

    core_failed=0
    check() {
        if ! grep -q "$1" "$log"; then
            echo "  MISSING: $2"
            core_failed=1
        fi
    }

    check 'Streaming mode enabled' 'data_recording.py replay'
    check 'Merging started correctly' 'data_merging.py'
    check 'Loaded .* waypoints' 'trajectory loading'
    check '\[STATE\] running' 'robot control loop'
    checkE() {
        if ! grep -qE "$1" "$log"; then
            echo "  MISSING: $2"
            core_failed=1
        fi
    }
    checkE 'Running on (http|all addresses)' 'web_interface.py'

    if [ "$core" = SSM ]; then
        check 'solver created successfully' 'CasADi/IPOPT solver'
    fi

    if grep -qE 'Traceback|terminate called after throwing|what\(\):' "$log"; then
        echo "  unhandled exception detected"
        core_failed=1
    fi

    if [ "$core_failed" -eq 0 ]; then
        echo "  RESULT: PASS"
    else
        echo "  RESULT: FAIL"
        status=1
        echo "  --- last log lines ---"
        tail -n 40 "$log" | sed 's/^/  /'
    fi
done

echo "==================================================================="
if [ "$status" -eq 0 ]; then
    echo "All cores passed."
else
    echo "One or more cores failed."
fi
exit "$status"
