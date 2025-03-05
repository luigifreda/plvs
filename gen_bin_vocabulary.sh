#!/usr/bin/env bash

# ====================================================
set -e

START_DIR=`pwd`

if [ ! -f "Vocabulary/ORBvoc.bin" ] && [ -f "Vocabulary/bin_vocabulary" ]; then

	. config.sh  # source configuration file 
	
	print_blue '================================================'
	print_blue "Building Vocabulary"
	print_blue '================================================'

	cd Vocabulary

	if [ ! -f ORBvoc.txt ]; then
		echo "Uncompress vocabulary ..."
		tar -xf ORBvoc.txt.tar.gz
	fi	

	if [ ! -f ORBvoc.bin ] && [ -f bin_vocabulary ]; then
		echo "Converting vocabulary to binary version"
		./bin_vocabulary
	fi

	echo "done"
fi

cd $START_DIR