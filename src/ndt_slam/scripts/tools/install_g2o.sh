#!/bin/bash

set -e

G2O_VERSION="2023_02_14"
G2O_REPO="https://github.com/RainerKuemmerle/g2o.git"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G2O_DIR="${SCRIPT_DIR}/../3rdparty/g2o"

echo "=========================================="
echo "Installing g2o ${G2O_VERSION}"
echo "=========================================="

if [ -d "${G2O_DIR}" ]; then
    echo "g2o directory already exists at ${G2O_DIR}"
    read -p "Do you want to remove and reinstall? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "${G2O_DIR}"
    else
        echo "Using existing g2o installation"
        exit 0
    fi
fi

echo "Cloning g2o repository..."
git clone --depth 1 --branch "${G2O_VERSION}" "${G2O_REPO}" "${G2O_DIR}"

cd "${G2O_DIR}"

echo "Creating build directory..."
mkdir -p build
cd build

echo "Configuring g2o with CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_CXX11=ON \
    -DBUILD_CSSOLVER=OFF

echo "Building g2o..."
make -j$(nproc)

echo "Installing g2o..."
sudo make install

echo "Updating library cache..."
sudo ldconfig

echo "=========================================="
echo "g2o installation completed successfully!"
echo "=========================================="
