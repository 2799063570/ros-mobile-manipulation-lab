// SPDX-License-Identifier: AGPL-3.0-only
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <aubo_perception/YoloDetectionArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "obb.hpp"

#if NV_TENSORRT_MAJOR < 10
#error This node requires TensorRT 10 or later.
#endif

namespace gc_yolo {
namespace {

void check_cuda(cudaError_t status, const char *operation) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

class TrtLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *message) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cerr << "TensorRT: " << message << '\n';
    }
  }
};

class Model {
public:
  explicit Model(const std::string &path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    const auto length = file.tellg();
    if (!file || length <= 0) {
      throw std::runtime_error("Cannot read engine: " + path);
    }
    std::vector<char> plan(static_cast<size_t>(length));
    file.seekg(0);
    if (!file.read(plan.data(), static_cast<std::streamsize>(plan.size()))) {
      throw std::runtime_error("Incomplete engine file: " + path);
    }
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("Could not create TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(plan.data(), plan.size()));
    if (!engine_) {
      throw std::runtime_error("Could not load engine; build it with trtexec on the target machine");
    }
    constexpr auto input_name = "images";
    constexpr auto output_name = "output0";
    if (engine_->getNbIOTensors() != 2 || engine_->getTensorIOMode(input_name) != nvinfer1::TensorIOMode::kINPUT ||
        engine_->getTensorIOMode(output_name) != nvinfer1::TensorIOMode::kOUTPUT) {
      throw std::runtime_error("Expected one input (images) and one output (output0)");
    }
    for (const auto *name : {input_name, output_name}) {
      if (engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT ||
          engine_->getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE ||
          engine_->getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR) {
        throw std::runtime_error(std::string(name) + " must use linear FP32 GPU I/O");
      }
    }
    const auto input = engine_->getTensorShape(input_name);
    const auto output = engine_->getTensorShape(output_name);
    constexpr auto max_int = std::numeric_limits<int>::max();
    if (input.nbDims != 4 || input.d[0] != 1 || input.d[1] != 3 || input.d[2] < 1 || input.d[2] > max_int ||
        input.d[3] != input.d[2] || output.nbDims != 3 || output.d[0] != 1 || output.d[1] <= 5 ||
        output.d[1] > max_int || output.d[2] < 1 || output.d[2] > max_int) {
      throw std::runtime_error("Expected fixed square NCHW input and [1, 5 + classes, anchors] OBB output");
    }
    size_ = static_cast<int>(input.d[2]);
    classes_ = static_cast<int>(output.d[1] - 5);
    anchors_ = static_cast<int>(output.d[2]);
    input_.resize(3 * static_cast<size_t>(size_) * size_);
    output_.resize(static_cast<size_t>(output.d[1]) * anchors_);

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("Could not create TensorRT execution context");
    }
    cudaStream_t stream = nullptr;
    check_cuda(cudaStreamCreate(&stream), "cudaStreamCreate");
    stream_.reset(stream);
    void *buffer = nullptr;
    check_cuda(cudaMalloc(&buffer, input_.size() * sizeof(float)), "cudaMalloc input");
    device_input_.reset(buffer);
    check_cuda(cudaMalloc(&buffer, output_.size() * sizeof(float)), "cudaMalloc output");
    device_output_.reset(buffer);
    if (!context_->setTensorAddress(input_name, device_input_.get()) ||
        !context_->setTensorAddress(output_name, device_output_.get())) {
      throw std::runtime_error("Could not bind TensorRT buffers");
    }
  }

  int classes() const { return classes_; }
  int image_size() const { return size_; }

  std::vector<Box> predict(const cv::Mat &image, float conf, float iou, size_t max_detections) {
    const double scale = std::min(static_cast<double>(size_) / image.cols, static_cast<double>(size_) / image.rows);
    const int width = std::max(1, static_cast<int>(std::nearbyint(image.cols * scale)));
    const int height = std::max(1, static_cast<int>(std::nearbyint(image.rows * scale)));
    const int left = (size_ - width) / 2;
    const int top = (size_ - height) / 2;
    cv::resize(image, resized_, {width, height}, 0, 0, cv::INTER_LINEAR);
    cv::copyMakeBorder(resized_, padded_, top, size_ - height - top, left, size_ - width - left,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    const size_t plane = static_cast<size_t>(size_) * size_;
    for (int row = 0; row < size_; ++row) {
      const auto *pixels = padded_.ptr<cv::Vec3b>(row);
      for (int column = 0; column < size_; ++column) {
        const size_t index = static_cast<size_t>(row) * size_ + column;
        input_[index] = pixels[column][2] / 255.0f;
        input_[index + plane] = pixels[column][1] / 255.0f;
        input_[index + 2 * plane] = pixels[column][0] / 255.0f;
      }
    }
    auto stream = stream_.get();
    check_cuda(cudaMemcpyAsync(device_input_.get(), input_.data(), input_.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream), "copy input");
    if (!context_->enqueueV3(stream)) {
      throw std::runtime_error("TensorRT enqueue failed");
    }
    check_cuda(cudaMemcpyAsync(output_.data(), device_output_.get(), output_.size() * sizeof(float),
                               cudaMemcpyDeviceToHost, stream), "copy output");
    check_cuda(cudaStreamSynchronize(stream), "synchronize inference");
    auto detections = decode_obb(output_.data(), anchors_, classes_, conf, iou, max_detections);
    for (auto &box : detections) {
      to_original_image(box, scale, left, top, image.cols, image.rows);
    }
    return detections;
  }

private:
  struct FreeDevice {
    void operator()(void *buffer) const { cudaFree(buffer); }
  };
  struct DestroyStream {
    void operator()(cudaStream_t stream) const { cudaStreamDestroy(stream); }
  };

  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  cv::Mat resized_, padded_;
  std::vector<float> input_, output_;
  std::unique_ptr<void, FreeDevice> device_input_, device_output_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::unique_ptr<std::remove_pointer_t<cudaStream_t>, DestroyStream> stream_;
  int size_{}, classes_{}, anchors_{};
};

class ObbNode {
public:
  ObbNode() {
    ros::NodeHandle params("~");
    std::string engine_path, image_topic, detections_topic, annotated_topic;
    params.param<std::string>("engine_path", engine_path, "");
    params.param<std::string>("image_topic", image_topic, "/camera/color/image_raw");
    params.param<std::string>("detections_topic", detections_topic, "/yolo/detections");
    params.param<std::string>("annotated_image_topic", annotated_topic, "/yolo/annotated_image");
    if (engine_path.empty()) {
      throw std::invalid_argument("Set engine_path to a TensorRT engine generated by trtexec");
    }
    double confidence, iou, rate;
    int gpu_device, expected_image_size;
    bool publish_annotated;
    params.param("confidence", confidence, 0.25);
    params.param("iou", iou, 0.70);
    params.param("inference_rate", rate, 10.0);
    params.param("max_detections", max_detections_, 100);
    params.param("gpu_device", gpu_device, 0);
    params.param("expected_image_size", expected_image_size, 0);
    params.param("publish_annotated_image", publish_annotated, true);
    params.param("class_names", class_names_, std::vector<std::string>{"bottle", "can", "box"});
    if (!(confidence >= 0 && confidence <= 1 && iou > 0 && iou <= 1 && std::isfinite(rate) && rate >= 0) ||
        max_detections_ <= 0 || gpu_device < 0) {
      throw std::invalid_argument("Invalid confidence, iou, inference_rate, max_detections or gpu_device");
    }
    conf_ = static_cast<float>(confidence);
    iou_ = static_cast<float>(iou);
    minimum_interval_ = rate > 0 ? 1.0 / rate : 0;
    check_cuda(cudaSetDevice(gpu_device), "cudaSetDevice");
    model_ = std::make_unique<Model>(engine_path);
    if (expected_image_size != 0 && model_->image_size() != expected_image_size) {
      throw std::invalid_argument("Engine input size does not match expected_image_size");
    }
    if (class_names_.size() != static_cast<size_t>(model_->classes()) ||
        std::any_of(class_names_.begin(), class_names_.end(), [](const std::string &name) { return name.empty(); })) {
      throw std::invalid_argument("class_names must contain one nonempty name per engine class, in training order");
    }
    publisher_ = node_.advertise<aubo_perception::YoloDetectionArray>(detections_topic, 1);
    if (publish_annotated) {
      annotated_ = node_.advertise<sensor_msgs::Image>(annotated_topic, 1);
    }
    subscriber_ = node_.subscribe(image_topic, 1, &ObbNode::on_image, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("TensorRT OBB: " << image_topic << " -> " << detections_topic << " (" << engine_path << ")");
  }

private:
  void on_image(const sensor_msgs::ImageConstPtr &message) {
    const auto now = ros::SteadyTime::now();
    if (!last_inference_.isZero() && (now - last_inference_).toSec() < minimum_interval_) {
      return;
    }
    last_inference_ = now;
    try {
      const auto image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
      if (image->image.empty()) {
        throw std::runtime_error("Empty input image");
      }
      const auto boxes = model_->predict(image->image, conf_, iou_, max_detections_);
      aubo_perception::YoloDetectionArray result;
      result.header = message->header;
      result.detections.reserve(boxes.size());
      for (const auto &box : boxes) {
        auto &detection = result.detections.emplace_back();
        detection.class_id = box.class_id;
        detection.class_name = class_names_[box.class_id];
        detection.confidence = box.score;
        detection.center_x = box.x;
        detection.center_y = box.y;
        detection.width = box.width;
        detection.height = box.height;
        detection.angle = box.angle;
        detection.orientation_valid = true;
      }
      publisher_.publish(result);
      if (annotated_ && annotated_.getNumSubscribers() > 0) {
        cv::Mat canvas = image->image.clone();
        for (const auto &box : boxes) {
          cv::Point2f corners[4];
          cv::RotatedRect({box.x, box.y}, {box.width, box.height}, box.angle * 57.2957795131f).points(corners);
          for (int i = 0; i < 4; ++i) {
            cv::line(canvas, corners[i], corners[(i + 1) % 4], {0, 255, 0}, 2, cv::LINE_AA);
          }
          cv::putText(canvas, class_names_[box.class_id], corners[1], cv::FONT_HERSHEY_SIMPLEX, 0.5,
                      {0, 255, 0}, 1, cv::LINE_AA);
        }
        annotated_.publish(cv_bridge::CvImage(message->header, sensor_msgs::image_encodings::BGR8, canvas).toImageMsg());
      }
    } catch (const std::exception &error) {
      ROS_ERROR_THROTTLE(2.0, "TensorRT OBB failed: %s", error.what());
    }
  }

  ros::NodeHandle node_;
  std::unique_ptr<Model> model_;
  ros::Publisher publisher_, annotated_;
  ros::Subscriber subscriber_;
  std::vector<std::string> class_names_;
  ros::SteadyTime last_inference_;
  double minimum_interval_{};
  int max_detections_{};
  float conf_{}, iou_{};
};

}  // namespace
}  // namespace gc_yolo

int main(int argc, char **argv) {
  ros::init(argc, argv, "tensorrt_yolo");
  try {
    gc_yolo::ObbNode node;
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL("Cannot start TensorRT YOLO: %s", error.what());
    return 1;
  }
  return 0;
}
