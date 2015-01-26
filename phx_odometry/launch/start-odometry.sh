#!/usr/bin/env bash

hostname=$(uname -n)
echo Starting launch file with env-loader for hostname $hostname
roslaunch phoenix_odometry raspi.launch hostname:=$hostname
