#!/bin/bash
# Usage: ./generate-python comms

$HOME/code/binder/build/source/binder \
	--root-module=pymvsim_$1 \
	--prefix $1/python/generated-sources/ \
	--bind pymvsim_$1 \
	-config python.conf \
	$1/python/all_$1_headers.hpp \
	-- \
	-std=c++17 -DNDEBUG \
	-DMVSIM_HAS_PROTOBUF=1 \
	-DMVSIM_HAS_ZMQ=1 \
	-I$1/include \
	-I/home/jlblanco/code/mrpt/build/include/mrpt-configuration/ \
	-I/home/jlblanco/code/mrpt/libs/system/include/ \
	-I/home/jlblanco/code/mrpt/libs/core/include/ \
	-I/home/jlblanco/code/mrpt/libs/typemeta/include \
	-I/home/jlblanco/code/mrpt/libs/io/include/ \
	-I/home/jlblanco/code/mrpt/libs/serialization/include/ \
	-I/home/jlblanco/code/mrpt/libs/rtti/include/ \
	-I/home/jlblanco/code/mrpt/libs/serialization/include/ \
