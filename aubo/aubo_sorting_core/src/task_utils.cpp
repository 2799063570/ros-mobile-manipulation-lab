#include "task_utils.hpp"

#include <sstream>
#include <stdexcept>

namespace aubo_sorting_core
{
  namespace detail
  {
    bool wallSleep(double seconds, const std::atomic<bool> &stop_requested)
    {
      // 使用墙上时间，Gazebo 暂停 /clock 时仍能响应停止；每 20 ms 检查一次退出条件。
      const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(seconds);
      ros::WallRate rate(50.0);
      while (ros::ok() && !stop_requested.load() && ros::WallTime::now() < deadline)
        rate.sleep();
      return ros::ok() && !stop_requested.load();
    }

    std::string join(const std::vector<std::string> &values, const std::string &separator)
    {
      // 分隔符仅插入元素之间，空数组返回空字符串。
      std::ostringstream stream;
      for (std::size_t index = 0; index < values.size(); ++index)
      {
        if (index)
          stream << separator;
        stream << values[index];
      }
      return stream.str();
    }

    std::string jsonEscape(const std::string &value) // 转义 JSON 字符串值
    {
      std::ostringstream stream;
      for (const unsigned char character : value)
      {
        switch (character)
        {
        case '"':
          stream << "\\\"";
          break;
        case '\\':
          stream << "\\\\";
          break;
        case '\b':
          stream << "\\b";
          break;
        case '\f':
          stream << "\\f";
          break;
        case '\n':
          stream << "\\n";
          break;
        case '\r':
          stream << "\\r";
          break;
        case '\t':
          stream << "\\t";
          break;
        default:
          if (character < 0x20)
          {
            stream << "\\u00";
            const char *digits = "0123456789abcdef";
            stream << digits[(character >> 4) & 0x0f] << digits[character & 0x0f];
          }
          else
            stream << character;
        }
      }
      return stream.str();
    }

    double xmlNumber(const XmlRpc::XmlRpcValue &value)
    {
      if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        return static_cast<double>(value);
      if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
        return static_cast<int>(value);
      throw std::runtime_error("expected a numeric value");
    }

    std::vector<double> xmlVector(const XmlRpc::XmlRpcValue &value, int expected_size,
                                  const std::string &name)
    {
      if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != expected_size)
        throw std::runtime_error("'" + name + "' must contain " + std::to_string(expected_size) +
                                 " numbers");
      std::vector<double> result;
      result.reserve(expected_size);
      for (int index = 0; index < expected_size; ++index)
        result.push_back(xmlNumber(value[index]));
      return result;
    }
  } // namespace detail
} // namespace aubo_sorting_core
