#!/bin/bash

# Step 1: Run colcon build
echo "[Info] Running colcon build..."
colcon build
BUILD_RESULT=$?

# Step 2: Check if the build was successful
if [ $BUILD_RESULT -ne 0 ]; then
    echo "[Error] colcon build failed. Aborting shebang fix."
    exit 1
fi

echo "[Success] colcon build completed. Proceeding to fix shebangs."

# Step 3: Set the path to the Python interpreter in the virtual environment
VENV_PATH="/home/pinky/venv/pinky/bin/python"

# Step 4: List of target files to update
TARGET_FILES=(
    install/nuri_bot/lib/nuri_bot/main_node
    install/nuri_bot/lib/nuri_bot/cam_node
    install/nuri_bot/lib/nuri_bot/stt_node
    install/nuri_bot/lib/nuri_bot/tts_node
)

# Step 5: Iterate over target files and fix shebang
for TARGET_FILE in "${TARGET_FILES[@]}"; do
    if [ -f "$TARGET_FILE" ]; then
        echo "[Success] Fixing shebang for $TARGET_FILE"
        sed -i "1s|^#!.*|#!$VENV_PATH|" "$TARGET_FILE"
    else
        echo "[Fail] Target launcher not found: $TARGET_FILE"
    fi
done
