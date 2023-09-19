#!/usr/bin/env bash

# https://serverfault.com/questions/897756/delete-a-file-that-was-created-with-fallocate

SWAP_FILE_NAME=$1
if [[ ! -n "$SWAP_FILE_NAME" ]]; then
    SWAP_FILE_NAME="/swapfile"	
fi
echo "removing swap file: $SWAP_FILE_NAME"

set -e # exit on first error 

sudo swapoff $SWAP_FILE_NAME

sudo rm $SWAP_FILE_NAME

sudo swapon --show

df -H


