#!/usr/bin/python3
"""直接运行专用 Gazebo 抓放采集工作流的 ROS 节点入口。"""
import sys
from aubo_vla.grasp_workflow import main

if __name__ == '__main__':
    sys.exit(main())
