#!/bin/bash

source ~/astro_ws_pas/install/setup.bash

unset RMW_IMPLEMENTATION
unset ZENOH_CONFIG_OVERRIDE

export ASTRO_NUM=5
export ROS_DOMAIN_ID=$ASTRO_NUM


echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Zenoh: disabled"

