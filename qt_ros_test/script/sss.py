#!/usr/bin/env python
# coding=utf-8

import subprocess
import os
import shlex
import sys


current_work_dir = os.path.dirname(os.path.abspath(__file__))  # 当前文件所在的目录
cmd1="expect "+current_work_dir+'/../src'
cmd2="expect "+current_work_dir+'/initssh_2.exp'
print(cmd1)
#$args1=os.system(cmd1)
#args1=os.system(cmd2)

sys.exit(0)
