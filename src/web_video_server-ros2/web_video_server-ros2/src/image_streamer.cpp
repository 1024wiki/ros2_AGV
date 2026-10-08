// ============================================================================
// 文件 : image_streamer.cpp
// 功能 : 流对象基类的实现——模板方法模式的骨架载体
//
// 本文件实现两个类：
//   ImageStreamer（顶层抽象产品的公共部分）：保存请求/连接/节点，解析 topic_；
//   ImageTransportImageStreamer（中间层模板骨架）：订阅→解码→缩放→首帧初始化→编码推送。
// 只加注释，不改任何源代码逻辑。
// ============================================================================
#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>   // ROS Image <-> OpenCV Mat 转换
#include <iostream>

namespace web_video_server
{

// ----------------------------------------------------------------------------
// ImageStreamer 构造：保存本次 HTTP 请求三要素，解析要订阅的话题名
// request：含 topic/quality/width 等 URL 参数；connection：本次 HTTP 连接；nh：ROS 节点
// 初始化 inactive_=false（默认活跃），再从 URL 取 topic 参数存入 topic_
// ----------------------------------------------------------------------------
ImageStreamer::ImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
    request_(request), connection_(connection), nh_(nh), inactive_(false)
{
  // 从 URL 查询串取 topic（如 /camera/color/image_raw），缺省为空字符串
  topic_ = request.get_query_param_value_or_default("topic", "");
}

// 基类析构：无资源释放（子类析构负责标记失效与释放编码上下文）
ImageStreamer::~ImageStreamer()
{
}

// ----------------------------------------------------------------------------
// 中间层构造：先调基类构造，再解析本层私有参数
// it_(nh)：image_transport 转运层（支持 raw/compressed 等传输方式协商）
// initialized_=false：首帧尚未初始化（FFmpeg 管线等首帧到达才建）
// width/height 默认 -1 表示“保持原尺寸”；invert 表示是否旋转180度；
// default_transport 默认 raw（可经 URL 切 compressed 转运）
// ----------------------------------------------------------------------------
ImageTransportImageStreamer::ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
  ImageStreamer(request, connection, nh), it_(nh), initialized_(false)
{
  // 输出宽高：URL 参数 width/height，-1 = 不缩放保持输入尺寸
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  // 是否翻转：URL 含 invert 参数即旋转180度（相机倒装时用）
  invert_ = request.has_query_param("invert");
  // 传输提示：默认 raw（也可指定 compressed，走 image_transport 转运协商）
  default_transport_ = request.get_query_param_value_or_default("default_transport", "raw");
}

ImageTransportImageStreamer::~ImageTransportImageStreamer()
{
}

// ----------------------------------------------------------------------------
// start：订阅 ROS 图像话题（模板骨架入口）
// 流程：先查话题是否存在（存在才置活跃），再经 image_transport 订阅，回调为 imageCallback
// 注意：即使话题暂时不存在也照常 subscribe（话题稍后出现会自动连上）；
// inactive_ 初始置 true，仅当在系统话题列表中找到目标话题才置 false
// ----------------------------------------------------------------------------
void ImageTransportImageStreamer::start()
{
  // 按 default_transport_ 生成转运提示（如 raw/compressed）
  image_transport::TransportHints hints(nh_.get(), default_transport_);
  // 获取系统全部话题及类型，用于校验目标话题是否存在
  auto tnat = nh_->get_topic_names_and_types();
  inactive_ = true;   // 先假设失效，找到话题再激活
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // explicitly avoid topics with more than one type
      // 明确避开“一话题多类型”的异常情况（这种话题不处理，直接跳出）
      break;
    }
    auto & topic_name = topic_and_types.first;
    // 匹配话题名：兼容带/不带前导斜杠两种写法
    if(topic_name == topic_ || (topic_name.find("/") == 0 && topic_name.substr(1) == topic_)){
      inactive_ = false;   // 找到目标话题，标记活跃
      break;
    }
  }
  // 订阅话题，队列 1（只关心最新帧），回调为本类 imageCallback
  image_sub_ = it_.subscribe(topic_, 1, &ImageTransportImageStreamer::imageCallback, this, &hints);
}

// 默认空实现的首帧初始化钩子：Mjpeg/Png 无需建管线；Libav 重写它建 FFmpeg 管线
void ImageTransportImageStreamer::initialize(const cv::Mat &)
{
}

// ----------------------------------------------------------------------------
// restreamFrame：限速/保活重推（模板骨架通用实现）
// 调用方：WebVideoServer::restreamFrames 定时遍历所有活跃流
// 语义：若距上次推送已超过 max_age，才把缓存的最新帧再推一次；
// 注意注释：此处故意不更新 last_frame（保持旧值，避免影响下次判断）
// ----------------------------------------------------------------------------
void ImageTransportImageStreamer::restreamFrame(double max_age)
{
  // 未激活或首帧尚未初始化（管线未建）则直接返回
  if (inactive_ || !initialized_ )
    return;
  try {
    // 距上次帧时间超过 max_age 才重推
    if ( last_frame + rclcpp::Duration::from_seconds(max_age) < nh_->now() ) {
      boost::mutex::scoped_lock lock(send_mutex_);   // 加锁保护发送与缓存
      sendImage(output_size_image, nh_->now() ); // don't update last_frame, it may remain an old value.
    }
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    RCLCPP_DEBUG(nh_->get_logger(), "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    // TODO THROTTLE with 30
    RCLCPP_ERROR(nh_->get_logger(), "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    // TODO THROTTLE with 30
    RCLCPP_ERROR(nh_->get_logger(), "exception");
    inactive_ = true;
    return;
  }
}

// ----------------------------------------------------------------------------
// imageCallback：模板方法骨架核心（每帧图像到达触发，不可被子类重写）
// 固定流程：解码→缩放/翻转→加锁缓存→首帧初始化钩子→调子类 sendImage 编码推送
// 两个留白点：initialize()（Libav 建 FFmpeg 管线）与 sendImage()（各编码实现）
// ----------------------------------------------------------------------------
void ImageTransportImageStreamer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  // 已失效（浏览器断开或此前异常）则直接丢弃本帧
  if (inactive_)
    return;

  cv::Mat img;
  try
  {
    // ---- 步骤 1：ROS Image → OpenCV Mat（按像素编码分类处理） ----
    if (msg->encoding.find("F") != std::string::npos)
    {
      // scale floating point images
      // 浮点图（如 32FC1 深度）：按最大值归一化到 0~255 便于显示
      cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
      cv::Mat_<float> float_image = float_image_bridge;
      double max_val;
      cv::minMaxIdx(float_image, 0, &max_val);   // 求全图最大值

      if (max_val > 0)
      {
        float_image *= (255 / max_val);   // 线性拉伸到 0~255
      }
      img = float_image;
    }
    else if(msg->encoding.find("16") != std::string::npos){
        // 16 位图（如 16UC1）：保持原位深直接拷贝
        img = cv_bridge::toCvCopy(msg, "16UC1")->image;
    }
    else{
        // Convert to OpenCV native BGR color
        // 普通彩色图：统一转 OpenCV 原生 BGR
        img = cv_bridge::toCvCopy(msg, "bgr8")->image;
      }

    // ---- 步骤 2：确定输出尺寸（URL 没指定则保持输入尺寸） ----
    int input_width = img.cols;
    int input_height = img.rows;

    if (output_width_ == -1)
      output_width_ = input_width;    // -1 表示不缩放
    if (output_height_ == -1)
      output_height_ = input_height;

    if (invert_)
    {
      // Rotate 180 degrees
      // 上下翻一次 + 左右翻一次 = 等效旋转180度（相机倒装时用）
      cv::flip(img, img, false);
      cv::flip(img, img, true);
    }

    boost::mutex::scoped_lock lock(send_mutex_); // protects output_size_image
    // 加锁保护输出缓存（restreamFrame 可能并发读取）
    if (output_width_ != input_width || output_height_ != input_height)
    {
      // 需要缩放：按 URL 指定尺寸重采样
      cv::Mat img_resized;
      cv::Size new_size(output_width_, output_height_);
      cv::resize(img, img_resized, new_size);
      output_size_image = img_resized;
    }
    else
    {
      // 尺寸一致：直接复用（浅拷贝共享数据）
      output_size_image = img;
    }

    // ---- 步骤 3：首帧初始化钩子（只执行一次） ----
    if (!initialized_)
    {
      initialize(output_size_image);   // Mjpeg/Png 空实现；Libav 在此建 FFmpeg 管线并发 HTTP 头
      initialized_ = true;
    }

    // ---- 步骤 4：调子类编码推送（模板留白点） ----
    last_frame = nh_->now();                  // 更新末帧时间戳（限速判断用）
    sendImage(output_size_image, last_frame ); // 纯虚函数：Mjpeg/Png/Libav 各自实现

  }
  // ---- 异常处理：任何环节出错都标记失效，等待 0.5s 清理器回收 ----
  catch (cv_bridge::Exception &e)
  {
    // TODO THROTTLE with 30
    // cv_bridge 转换失败（如编码名非法）：记错并失效
    RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (cv::Exception &e)
  {
    // TODO THROTTLE with 30
    // OpenCV 异常（如 resize 尺寸非法）：记错并失效
    RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    // socket 系统异常：多为浏览器断开，用 DEBUG 级别避免刷屏，并置失效
    RCLCPP_DEBUG(nh_->get_logger(), "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    // TODO THROTTLE with 30
    // 其他标准异常：记错并失效
    RCLCPP_ERROR(nh_->get_logger(), "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    // TODO THROTTLE with 30
    // 未知异常：兜底失效，保证流对象一定能被回收，不泄漏订阅
    RCLCPP_ERROR(nh_->get_logger(), "exception");
    inactive_ = true;
    return;
  }
}

}
