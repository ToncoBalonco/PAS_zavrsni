#!/bin/bash

source ~/astro_ws_pas/install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ASTRO_NUM=5
export ROS_DOMAIN_ID=$ASTRO_NUM
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.0.15:7447"]'

echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Zenoh endpoint: 192.168.0.15:7447"


