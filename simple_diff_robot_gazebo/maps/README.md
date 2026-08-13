# Maps

建图后运行 `map_saver.launch`，此目录会生成 `my_map.pgm` 和 `my_map.yaml`。

`navigation.launch` 默认读取 `my_map.yaml`，因此首次导航前必须先建图并保存。

如果包安装在 `/opt/ros` 或其他只读目录，请指定可写的绝对路径：

```bash
roslaunch simple_diff_robot_gazebo map_saver.launch map_name:=/home/用户名/maps/my_map
```

导航时对应指定：

```bash
roslaunch simple_diff_robot_gazebo navigation.launch map_file:=/home/用户名/maps/my_map.yaml
```
