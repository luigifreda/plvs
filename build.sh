#!/usr/bin/env bash

. config.sh  # source configuration file and utils 

./build_thirdparty.sh "$@"

./build_plvs.sh "$@"

./gen_bin_vocabulary.sh
