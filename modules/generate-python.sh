#!/bin/bash
# Usage: ./generate-python.sh comms

PYBIND11_VERSION=$(dpkg -s pybind11-dev | grep '^Version:' | cut -d " " -f2)
SYSTEM_PYBIND11_MM_VERSION=$(echo $PYBIND11_VERSION | cut -d. -f1).$(echo $PYBIND11_VERSION | cut -d. -f2)

PYBIND11_MM_VERSION=${PYBIND11_MM_VERSION:-$SYSTEM_PYBIND11_MM_VERSION}

echo "System PYBIND11_VERSION: $PYBIND11_VERSION (Used for wrapper: $PYBIND11_MM_VERSION)"

mkdir -p $1/python/generated-sources-pybind${PYBIND11_MM_VERSION}

$HOME/code/binder-for-pybind${PYBIND11_MM_VERSION}/build/source/binder \
	--root-module=pymvsim_$1 \
	--prefix $1/python/generated-sources-pybind${PYBIND11_MM_VERSION}/ \
	--bind pymvsim_$1 \
	-config python.conf \
	$1/python/all_$1_headers.hpp \
	-- \
	-std=c++17 -DNDEBUG \
	-DMVSIM_HAS_PROTOBUF=1 \
	-DMVSIM_HAS_ZMQ=1 \
	-I$1/include \
	-I/home/jlblanco/code/mrpt/build-Release/include/mrpt-configuration/ \
	-I/home/jlblanco/code/mrpt/libs/system/include/ \
	-I/home/jlblanco/code/mrpt/libs/core/include/ \
	-I/home/jlblanco/code/mrpt/libs/typemeta/include \
	-I/home/jlblanco/code/mrpt/libs/io/include/ \
	-I/home/jlblanco/code/mrpt/libs/serialization/include/ \
	-I/home/jlblanco/code/mrpt/libs/rtti/include/ \
	-I/home/jlblanco/code/mrpt/libs/serialization/include/ \
	-I/home/jlblanco/code/mrpt/libs/containers/include/ \
