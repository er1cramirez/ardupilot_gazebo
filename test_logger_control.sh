#!/bin/bash

# DataLogger Transport Control Test Script

echo "=== DataLogger Transport Control Test ==="
echo

# Clean up old logs
echo "1. Cleaning up old log files..."
rm -f ~/tmp/drone_sim_logs/*.csv
echo "   Done."
echo

# Start simulation in background
echo "2. Starting simulation with logging disabled..."
cd /home/eric/droneSim_ws/src/ardupilot_gazebo
gz sim -v4 -r worlds/iris_aruco_runway.sdf &
SIM_PID=$!

# Wait for simulation to start
echo "   Waiting 10 seconds for simulation to initialize..."
sleep 10

# Check initial state (should be empty except headers)
echo "3. Checking initial log state (should be header only)..."
if [ -f ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv ]; then
    echo "   Log file exists. Content:"
    head -2 ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv
    LINE_COUNT=$(wc -l < ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv)
    echo "   Line count: $LINE_COUNT (should be 1 - header only)"
else
    echo "   No log file found yet."
fi
echo

# Enable logging
echo "4. Enabling logging via transport..."
gz topic -t "/data_logger/enable" -m gz.msgs.Boolean -p "data: true"
echo "   Logging enabled. Waiting 5 seconds for data collection..."
sleep 5

# Check if data is being logged
echo "5. Checking if data is being logged..."
LINE_COUNT=$(wc -l < ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv)
echo "   Line count: $LINE_COUNT (should be > 1 if logging)"
if [ $LINE_COUNT -gt 1 ]; then
    echo "   ✓ SUCCESS: Data is being logged!"
    echo "   Sample data:"
    tail -3 ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv
else
    echo "   ✗ FAILED: No data logged"
fi
echo

# Disable logging
echo "6. Disabling logging..."
gz topic -t "/data_logger/enable" -m gz.msgs.Boolean -p "data: false"
echo "   Logging disabled. Waiting 3 seconds..."
LINES_BEFORE=$(wc -l < ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv)
sleep 3
LINES_AFTER=$(wc -l < ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv)

if [ $LINES_BEFORE -eq $LINES_AFTER ]; then
    echo "   ✓ SUCCESS: Logging stopped (line count unchanged: $LINES_BEFORE)"
else
    echo "   ✗ FAILED: Logging still active (lines: $LINES_BEFORE → $LINES_AFTER)"
fi
echo

# Test reset functionality
echo "7. Testing reset functionality..."
echo "   Lines before reset: $(wc -l < ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv)"
gz topic -t "/data_logger/reset" -m gz.msgs.Empty
sleep 2
echo "   Lines after reset: $(wc -l < ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv)"
echo "   Content after reset:"
cat ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv
echo

# Re-enable logging to test reset worked
echo "8. Re-enabling logging to verify reset worked..."
gz topic -t "/data_logger/enable" -m gz.msgs.Boolean -p "data: true"
sleep 3
FINAL_LINES=$(wc -l < ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv)
echo "   Final line count: $FINAL_LINES"
if [ $FINAL_LINES -gt 1 ]; then
    echo "   ✓ SUCCESS: Logging resumed after reset!"
    echo "   Sample data (should start from timestamp 0):"
    head -3 ~/tmp/drone_sim_logs/iris_with_standoffs_log.csv
else
    echo "   ✗ FAILED: Logging did not resume"
fi
echo

# Cleanup
echo "9. Stopping simulation..."
kill $SIM_PID
wait $SIM_PID 2>/dev/null
echo "   Done."
echo

echo "=== Test Complete ==="
echo "Transport control topics available:"
echo "  Enable/Disable: gz topic -t '/data_logger/enable' -m gz.msgs.Boolean -p 'data: true|false'"
echo "  Reset logs:     gz topic -t '/data_logger/reset' -m gz.msgs.Empty"
