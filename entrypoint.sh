#!/bin/bash

sudo mkdir /root/logs
cd /root/collaborative_payload_transport
source devel/setup.bash
roslaunch collab $@

