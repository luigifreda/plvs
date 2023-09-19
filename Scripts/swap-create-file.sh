#!/usr/bin/env bash

# https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-16-04


SWAP_FILE_NAME=$1
if [[ ! -n "$SWAP_FILE_NAME" ]]; then
    SWAP_FILE_NAME="/swapfile"	
fi

SIZE_SWAP=$2
if [[ ! -n "$SIZE_SWAP" ]]; then
    SIZE_SWAP="20G"	
fi

echo "creating swap file: $SWAP_FILE_NAME of size $SIZE_SWAP"

set -e # exit on first error 

sudo swapon --show # verify the current status 

sudo fallocate -l $SIZE_SWAP $SWAP_FILE_NAME

ls -lh $SWAP_FILE_NAME # check 

sudo chmod 600 $SWAP_FILE_NAME # Make the file only accessible to root

ls -lh $SWAP_FILE_NAME  # verify permission 

sudo mkswap $SWAP_FILE_NAME # now mark the file as swap space

sudo swapon $SWAP_FILE_NAME

# setup the swappiness

sudo sysctl vm.swappiness=1

cat /proc/sys/vm/swappiness

# verify the current status

df -H

sudo swapon --show  




# N.B.: to remove the swap file run the script swapfile-remove.sh

