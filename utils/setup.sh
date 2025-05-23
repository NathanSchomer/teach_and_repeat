#!/bin/bash

if [ ! -d "./src/teach_and_repeat" ]; then
    echo "Error: ./src/teach_and_repeat does not exist."
    exit 1
fi

cd ./src/teach_and_repeat

echo "Updating git submodules recursively..."
git submodule update --init --recursive

# Copy modified files into dbow2 submodule folder
echo "Copying modified build files into dbow2 submodule..."
cp "./utils/dbow2/CMakeLists.txt" "./dbow2/CMakeLists.txt"
cp "./utils/dbow2/package.xml" "./dbow2/package.xml"

cd ../../

echo "Setup complete."
