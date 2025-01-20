#!/bin/bash
# Usage: ./generate-python.sh comms

PYBIND11_VERSION=$(dpkg -s pybind11-dev | grep '^Version:' | cut -d " " -f2)
SYSTEM_PYBIND11_MM_VERSION=$(echo $PYBIND11_VERSION | cut -d. -f1).$(echo $PYBIND11_VERSION | cut -d. -f2)

PYTHON_INCLUDE_PATH=$(python3 -c "from sysconfig import get_paths; print(get_paths()['include'])")

PYBIND11_MM_VERSION=${PYBIND11_MM_VERSION:-$SYSTEM_PYBIND11_MM_VERSION}

echo "System PYBIND11_VERSION: $PYBIND11_VERSION (Used for wrapper: $PYBIND11_MM_VERSION)"

mkdir -p $1/python/generated-sources-pybind

BINDER=$(command -v binder || echo "$HOME/code/binder/build/source/binder")

MRPT_PATH="${MRPT_PATH:-/home/jlblanco/code/mrpt}"

$BINDER \
	--root-module=pymvsim_$1 \
	--prefix $1/python/generated-sources-pybind/ \
	--bind pymvsim_$1 \
	-config python.conf \
	$1/python/all_$1_headers.hpp \
	-- \
	-std=c++17 -DNDEBUG \
	-DMVSIM_HAS_PROTOBUF=1 \
	-DMVSIM_HAS_ZMQ=1 \
	-DMVSIM_HAS_PYTHON=1 \
	-I$1/include \
	-I$PYTHON_INCLUDE_PATH \
	-I$MRPT_PATH/build-Release/include/mrpt-configuration/ \
	-I$MRPT_PATH/libs/system/include/ \
	-I$MRPT_PATH/libs/core/include/ \
	-I$MRPT_PATH/libs/typemeta/include \
	-I$MRPT_PATH/libs/io/include/ \
	-I$MRPT_PATH/libs/serialization/include/ \
	-I$MRPT_PATH/libs/rtti/include/ \
	-I$MRPT_PATH/libs/serialization/include/ \
	-I$MRPT_PATH/libs/containers/include/ \
