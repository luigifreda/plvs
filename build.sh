#!/usr/bin/env bash

. config.sh  # source configuration file and utils 

./gen_bin_vocabulary.sh

./build_thirdparty.sh "$@"

./build_plvs.sh "$@"

