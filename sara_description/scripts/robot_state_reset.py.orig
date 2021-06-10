#!/usr/bin/env python
import os

nodes = os.popen("rosnode list").readlines()
for i in range(len(nodes)):
    nodes[i] = nodes[i].replace("\n","")

for node in nodes:
    if node == "robot_state_publisher":
        os.system("rosnode kill "+ node)
#os.system("rosnode kill robot_state_publisher")
