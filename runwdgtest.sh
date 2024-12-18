#!/bin/bash

# Trap SIGINT (Ctrl+C) to handle interruptions
trap 'echo -e "\nCommand interrupted. Moving to the next test..."; continue' SIGINT

INACTIVITY_TIMEOUT=10

# Function to run the tests
run_tests() {
    for map in {1..30}; do
        for mode in 0 1; do
            for thread in 1 2 4 8; do
                # Skip unnecessary combinations if needed
                if [[ $mode -eq 0 && $thread -ne 1 ]]; then
                    continue
                fi

		if [[ $mode -ne 0 && $thread -eq 1 ]]; then
			continue
		fi

        # Command to execute
        command="bin/wdg_test $map $mode $thread"
        echo "Running: $command"

		stdbuf -oL $command | (
                    SECONDS=0
                    timeout_triggered=0
                    while IFS= read -r line; do
                        echo "$line"  # Print the line
                        SECONDS=0    # Reset the inactivity timer
                    done &

                    # Monitor the process
                    pid=$!
                    while kill -0 $pid 2>/dev/null; do
                        if (( SECONDS > INACTIVITY_TIMEOUT )); then
                            echo "No new output for $INACTIVITY_TIMEOUT seconds. Killing: $command"
                            kill -9 $pid 2>/dev/null
                            timeout_triggered=1
                            break
                        fi
                        sleep 1
                    done

                    # Ensure proper cleanup and exit from loop
                    wait $pid 2>/dev/null || true
                    if (( timeout_triggered == 1 )); then
                        echo "Command timed out."
                    fi
                    continue
                )
                echo
            done
        done
    done
}

# Start the tests
echo "Starting all tests. Press Ctrl+C to skip any command."
run_tests

echo "All tests completed."
