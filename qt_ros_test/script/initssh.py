#!/usr/bin/env python
# coding=utf-8

import subprocess
import os
import shlex
import sys


current_work_dir = os.path.dirname(os.path.abspath(__file__))  # 当前文件所在的目录
cmd1="expect "+current_work_dir+'/initssh_1.exp'
cmd2="expect "+current_work_dir+'/initssh_2.exp'
username=os.environ['USERNAME']
args=shlex.split("ssh-keygen -f '/home/"+username+"/.ssh/known_hosts' -R '192.168.0.100'")
p=subprocess.Popen(args,stdin=subprocess.PIPE)
args1=shlex.split("ssh-add -D")
p1=subprocess.Popen(args1,stdin=subprocess.PIPE)
args1=os.system(cmd1)
args1=os.system(cmd2)

sys.exit(0)
