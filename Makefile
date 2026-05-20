# Makefile for building the Collisions project on Linux using vcpkg + CMake + Ninja
# Usage:
#   make install-deps   # clones vcpkg (if needed) and installs manifest deps
#   make configure      # generate build files with CMake
#   make build          # build with cmake --build
#   make run            # run the produced binary
#   make clean          # remove build artifacts
# Variables you can override on the command line:
#   CONFIG=Debug or Release
#   VCPKG_ROOT (defaults to ~/vcpkg)

CONFIG ?= Debug
VCPKG_ROOT ?= $(HOME)/vcpkg
BUILD_DIR := build/linux-$(CONFIG)
CMAKE := cmake
NINJA := ninja
PROJECT := Collisions

.PHONY: all install-deps configure build run clean

all: install-deps configure build

install-deps: $(VCPKG_ROOT)/vcpkg
	@echo "Installing vcpkg packages from manifest (vcpkg.json)..."
	$(VCPKG_ROOT)/vcpkg install --x-manifest-root=. --triplet=x64-linux

# clone and bootstrap vcpkg if it doesn't exist
$(VCPKG_ROOT)/vcpkg:
	@echo "vcpkg not found at $(VCPKG_ROOT). Cloning..."
	git clone https://github.com/microsoft/vcpkg.git $(VCPKG_ROOT)
	$(VCPKG_ROOT)/bootstrap-vcpkg.sh
	@echo "vcpkg bootstrapped at $(VCPKG_ROOT)"
	@$(VCPKG_ROOT)/vcpkg --version || true

configure: $(BUILD_DIR)/CMakeCache.txt

$(BUILD_DIR)/CMakeCache.txt:
	@echo "Configuring project (CONFIG=$(CONFIG))"
	$(CMAKE) -S . -B $(BUILD_DIR) -G Ninja -DCMAKE_BUILD_TYPE=$(CONFIG) \
	  -DCMAKE_TOOLCHAIN_FILE=$(VCPKG_ROOT)/scripts/buildsystems/vcpkg.cmake

build: configure
	@echo "Building project"
	$(CMAKE) --build $(BUILD_DIR) --config $(CONFIG)

run: build
	@echo "Running binary"
	$(BUILD_DIR)/$(PROJECT)

clean:
	rm -rf $(BUILD_DIR)

# helper to reconfigure (force)
reconfigure:
	rm -f $(BUILD_DIR)/CMakeCache.txt
	$(MAKE) configure
