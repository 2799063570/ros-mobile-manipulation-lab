# ddynamic_reconfigure

`ddynamic_reconfigure` 是 [dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure) 的 C++ 扩展。它允许 ROS 节点无需编写 `.cfg` 文件，即可通过动态参数框架修改变量。

## 用法

直接绑定变量：

```cpp
#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

int main(int argc, char **argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
    ddynamic_reconfigure::DDynamicReconfigure ddr;
    int int_param = 0;
    ddr.registerVariable<int>("int_param", &int_param, "参数说明");
    ddr.publishServicesTopics();
    // GUI 或其他工具修改参数时，int_param 会自动同步更新。
    int_param = 10;  // 同时把动态参数界面中的值更新为 10。
    ros::spin();
    return 0;
}
```

通过回调修改变量：

```cpp
int global_int;

void paramCb(int new_value) {
    global_int = new_value;
    ROS_INFO("参数已修改");
}

// 初始值为 10，每次更新都会调用 paramCb。
ddr.registerVariable<int>("int_param", 10, boost::bind(paramCb, _1), "参数说明");
ddr.publishServicesTopics();
```

注册枚举变量：

```cpp
std::map<std::string, std::string> enum_map = {
    {"键 1", "值 1"}, {"键 2", "值 2"}
};
std::string enum_value = enum_map["键 1"];
ddr.registerEnumVariable<std::string>(
    "string_enum", &enum_value, "参数说明", enum_map);
ddr.publishServicesTopics();
```

在私有命名空间 `ddynamic_tutorials/other_namespace/int_param` 中注册变量：

```cpp
ros::NodeHandle nh("~/other_namespace");
ddynamic_reconfigure::DDynamicReconfigure ddr(nh);
int int_param = 0;
ddr.registerVariable<int>("int_param", &int_param, "参数说明");
ddr.publishServicesTopics();
```

也可以先创建 `DDynamicReconfigure` 指针，之后再用指定的 `NodeHandle` 初始化：

```cpp
std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr;
ros::NodeHandle nh("~/other_namespace");
ddr.reset(new ddynamic_reconfigure::DDynamicReconfigure(nh));
```

## 常见问题

### `registerVariable` 或 `registerEnumVariable` 出现未定义引用

这两个方法是模板方法，但实现未在头文件中公开。库只为 `int`、`bool`、`double` 和 `std::string` 提供了显式模板实例，请确认传入参数属于这些类型。
