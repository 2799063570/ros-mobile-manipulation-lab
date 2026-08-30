# Maps

本目录自带与默认 `worlds/zlab_map.world` 匹配的 `zlab_map.pgm` 和
`zlab_map.yaml`，可直接供 `navigation.launch` 使用。

重新建图后运行 `map_saver.launch`，默认会覆盖 `zlab_map.pgm` 和
`zlab_map.yaml`。如需保留自带地图，请通过 `map_name` 指定其他文件名。

如果包安装在 `/opt/ros` 或其他只读目录，请指定可写的绝对路径：

```bash
roslaunch simple_diff_robot_gazebo map_saver.launch map_name:=/home/用户名/maps/my_map
```

导航时对应指定：

```bash
roslaunch simple_diff_robot_gazebo navigation.launch map_file:=/home/用户名/maps/my_map.yaml
```
