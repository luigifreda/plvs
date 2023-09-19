#!/bin/bash
# from https://devtalk.nvidia.com/default/topic/1027819/jetson-tx2/object-detection-performance-jetson-tx2-slower-than-expected/

sudo nvpmodel -m 0
sudo ./jetson_clocks.sh
