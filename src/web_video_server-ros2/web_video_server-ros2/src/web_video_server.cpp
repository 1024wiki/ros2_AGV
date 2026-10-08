// ============================================================================
// 文件 : web_video_server.cpp
// 功能 : web_video_server 核心实现——把 ROS 图像话题通过 HTTP 推流到浏览器
//
// 工作原理：
//   1. 建立一个异步 HTTP 服务器（默认监听 0.0.0.0:8080）
//   2. 注册 4 个路由：/（话题列表）、/stream（视频流）、/snapshot（截图）、/stream_viewer（网页）
//   3. 浏览器请求某个图像话题时，动态创建"流"对象订阅该 ROS 话题
//   4. 图像一帧帧到来 → 编码 → 通过 HTTP 持续推送给浏览器
//
// 这是 RobotWebTools 开源项目（BSD 协议）的 ROS2 移植版，通用性强，
// 只要系统里有 sensor_msgs/Image 类型的话题，无需改代码即可推流。
// ============================================================================

#include <boost/foreach.hpp>                  // BOOST_FOREACH 宏（遍历容器）
#include <boost/lexical_cast.hpp>             // 类型转换（如 int → string）
#include <boost/algorithm/string/predicate.hpp>  // 字符串判断（starts_with/ends_with）
#include <chrono>
#include <vector>
#include <sensor_msgs/image_encodings.hpp>    // 图像编码常量
#include <opencv2/opencv.hpp>                 // OpenCV（图像编码用）

#include "web_video_server/web_video_server.h"      // 本类声明
#include "web_video_server/ros_compressed_streamer.h" // 转发压缩图（不重新编码）
#include "web_video_server/jpeg_streamers.h"         // MJPEG 流 + JPEG 截图
#include "web_video_server/png_streamers.h"          // PNG 流
#include "web_video_server/vp8_streamer.h"           // VP8 编码流
#include "web_video_server/h264_streamer.h"          // H.264 编码流
#include "web_video_server/vp9_streamer.h"           // VP9 编码流
#include "async_web_server_cpp/http_reply.hpp"       // HTTP 响应构造器

using namespace std::chrono_literals;   // 允许写 500ms、1s 这种字面量
using namespace boost::placeholders;    // _1 _2 _3 _4 占位符（bind 用）

namespace web_video_server
{

static bool __verbose;                 // 是否打印详细日志（构造时从参数读取）
static std::string __default_stream_type;  // 默认编码类型（默认 "mjpeg"）

// ============================================================================
// ros_connection_logger：HTTP 请求的"包装器"（中间件）
//
// 作用：包在真正处理函数外面，统一做两件事：
//   1. 打印日志（verbose 模式）
//   2. 捕获异常（防止某个请求处理出错导致整个服务器崩溃）
// ============================================================================
static bool ros_connection_logger(async_web_server_cpp::HttpServerRequestHandler forward,
                                  const async_web_server_cpp::HttpRequest &request,
                                  async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                  const char* end)
{
  if (__verbose)
  {
    // TODO reenable
    // RCLCPP_INFO(nh->get_logger(), "Handling Request: %s", request.uri.c_str());
  }
  try
  {
    forward(request, connection, begin, end);   // 调用真正的处理函数（handler_group_）
    return true;
  }
  catch (std::exception &e)
  {
    // TODO reenable
    // RCLCPP_WARN(nh->get_logger(), "Error Handling Request: %s", e.what());
    return false;   // 处理出错：返回 false，不传播异常
  }
  return false;
}

// ============================================================================
// 构造函数：读取参数 + 注册编码器 + 注册路由 + 创建 HTTP 服务器
// ============================================================================
WebVideoServer::WebVideoServer(rclcpp::Node::SharedPtr &nh, rclcpp::Node::SharedPtr &private_nh) :
    nh_(nh), handler_group_(
        async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found))
{
  // ---------- 1. 读取私有参数（从 private_nh 节点，可用 launch 参数覆盖） ----------
  rclcpp::Parameter parameter;
  // 端口号（默认 8080）
  if (private_nh->get_parameter("port", parameter)) {
    port_ = parameter.as_int();
  } else {
    port_ = 8080;
  }
  // 是否详细日志（默认 true）
  if (private_nh->get_parameter("verbose", parameter)) {
    __verbose = parameter.as_bool();
  } else {
    __verbose = true;
  }
  // 监听地址（默认 0.0.0.0 = 所有网卡，局域网内其他设备可访问）
  if (private_nh->get_parameter("address", parameter)) {
    address_ = parameter.as_string();
  } else {
    address_ = "0.0.0.0";
  }
  // HTTP 服务器线程数（默认 1）
  int server_threads;
  if (private_nh->get_parameter("server_threads", parameter)) {
    server_threads = parameter.as_int();
  } else {
    server_threads = 1;
  }
  // ROS 回调线程数（默认 2）
  if (private_nh->get_parameter("ros_threads", parameter)) {
    ros_threads_ = parameter.as_int();
  } else {
    ros_threads_ = 2;
  }
  // 推流频率（默认 -1 = 不限速，图像来多快推多快）
  if (private_nh->get_parameter("publish_rate", parameter)) {
    publish_rate_ = parameter.as_double();
  } else {
    publish_rate_ = -1.0;
  }
  // 默认编码类型（默认 mjpeg）
  if (private_nh->get_parameter("default_stream_type", parameter)) {
    __default_stream_type = parameter.as_string();
  } else {
    __default_stream_type = "mjpeg";
  }

  // ---------- 2. 注册 6 种编码器到 stream_types_ 注册表 ----------
  // key = URL 里的 type 参数，value = 对应编码器的"工厂"（用来创建具体流对象）
  stream_types_["mjpeg"] = boost::shared_ptr<ImageStreamerType>(new MjpegStreamerType());
  stream_types_["png"] = boost::shared_ptr<ImageStreamerType>(new PngStreamerType());
  stream_types_["ros_compressed"] = boost::shared_ptr<ImageStreamerType>(new RosCompressedStreamerType());
  stream_types_["vp8"] = boost::shared_ptr<ImageStreamerType>(new Vp8StreamerType());
  stream_types_["h264"] = boost::shared_ptr<ImageStreamerType>(new H264StreamerType());
  stream_types_["vp9"] = boost::shared_ptr<ImageStreamerType>(new Vp9StreamerType());

  // ---------- 3. 注册 4 个 URL 路由 ----------
  // 浏览器访问哪个路径，就调用对应的 handle_xxx 处理函数
  handler_group_.addHandlerForPath("/", boost::bind(&WebVideoServer::handle_list_streams, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/stream", boost::bind(&WebVideoServer::handle_stream, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/stream_viewer",
                                   boost::bind(&WebVideoServer::handle_stream_viewer, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/snapshot", boost::bind(&WebVideoServer::handle_snapshot, this, _1, _2, _3, _4));

  // ---------- 4. 创建 HTTP 服务器 ----------
  try
  {
    server_.reset(
        new async_web_server_cpp::HttpServer(address_, boost::lexical_cast<std::string>(port_),
                                             boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4),
                                             server_threads));
  }
  catch(boost::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Exception when creating the web server! %s:%d", address_.c_str(), port_);
    throw;
  }
}

WebVideoServer::~WebVideoServer()
{
}

// ============================================================================
// setup_cleanup_inactive_streams：启动一个 0.5 秒定时器，定期清理失效的流
// ============================================================================
void WebVideoServer::setup_cleanup_inactive_streams()
{
  std::function<void()> callback = std::bind(&WebVideoServer::cleanup_inactive_streams, this);
  cleanup_timer_ = nh_->create_wall_timer(500ms, callback);   // 每 500ms 调一次 cleanup_inactive_streams
}

// ============================================================================
// spin：启动服务器并进入事件循环（阻塞直到节点被关闭）
// ============================================================================
void WebVideoServer::spin()
{
  server_->run();   // 启动 HTTP 服务器（开始监听端口）
  RCLCPP_INFO(nh_->get_logger(), "Waiting For connections on %s:%d", address_.c_str(), port_);

  // 创建多线程执行器（用 ros_threads_ 个线程并行处理 ROS 回调，避免图像回调阻塞）
  rclcpp::executors::MultiThreadedExecutor spinner(rclcpp::ExecutorOptions(), ros_threads_);
  spinner.add_node(nh_);

  // 如果设置了 publish_rate_（限速推流），创建定时器周期性重推最新帧
  if ( publish_rate_ > 0 ) {
    nh_->create_wall_timer(1s / publish_rate_, [this](){restreamFrames(1.0 / publish_rate_);});
  }
  spinner.spin();   // 阻塞运行
  server_->stop();  // 退出时停止服务器
}

// ============================================================================
// restreamFrames：把所有活跃流的"最新一帧"重推出去（用于限速/保活）
// ============================================================================
void WebVideoServer::restreamFrames( double max_age )
{
  boost::mutex::scoped_lock lock(subscriber_mutex_);   // 加锁保护 image_subscribers_

  typedef std::vector<boost::shared_ptr<ImageStreamer> >::iterator itr_type;

  for (itr_type itr = image_subscribers_.begin(); itr < image_subscribers_.end(); ++itr)
    {
      (*itr)->restreamFrame( max_age );   // 每个流自己判断是否要重推
    }
}

// ============================================================================
// cleanup_inactive_streams：清理没有浏览器连接的流（释放订阅和资源）
// ============================================================================
void WebVideoServer::cleanup_inactive_streams()
{
  // try_to_lock：拿不到锁就跳过（避免和正在添加流的操作冲突）
  boost::mutex::scoped_lock lock(subscriber_mutex_, boost::try_to_lock);
  if (lock)
  {
    typedef std::vector<boost::shared_ptr<ImageStreamer> >::iterator itr_type;
    // std::partition：把"活跃的"挪到前面，"不活跃的"挪到后面，返回分界点
    itr_type new_end = std::partition(image_subscribers_.begin(), image_subscribers_.end(),
                                      !boost::bind(&ImageStreamer::isInactive, _1));
    if (__verbose)
    {
      for (itr_type itr = new_end; itr < image_subscribers_.end(); ++itr)
      {
        RCLCPP_INFO(nh_->get_logger(), "Removed Stream: %s", (*itr)->getTopic().c_str());
      }
    }
    // 删除所有不活跃的流（它们的析构函数会取消 ROS 订阅）
    image_subscribers_.erase(new_end, image_subscribers_.end());
  }
}

// ============================================================================
// handle_stream：处理 /stream 请求（核心！视频流，持续推流）
//
// URL 示例：/stream?topic=/camera/color/image_raw&type=mjpeg
//   ① type 参数：选编码器（mjpeg/png/ros_compressed/vp8/h264/vp9）
//   ② topic 参数：要订阅的 ROS 图像话题
// 流程：查编码器 → （ros_compressed 特殊处理）→ 创建流对象 → 订阅话题 → 持续推流
// ============================================================================
bool WebVideoServer::handle_stream(const async_web_server_cpp::HttpRequest &request,
                                   async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                   const char* end)
{
  // 取 type 参数，没传就用默认（mjpeg）
  std::string type = request.get_query_param_value_or_default("type", __default_stream_type);
  // 判断这个 type 是否在编码器注册表里
  if (stream_types_.find(type) != stream_types_.end())
  {
    // 取 topic 参数（要订阅的话题名）
    std::string topic = request.get_query_param_value_or_default("topic", "");

    // ---------- ros_compressed 特殊处理：查找压缩话题是否存在 ----------
    // ros_compressed 需要订阅 "<topic>/compressed" 这个压缩话题；
    // 如果系统里没有这个压缩话题（相机没配压缩），就回退成 mjpeg（订阅原图自己编码）
    if (type == std::string("ros_compressed"))
    {
      std::string compressed_topic_name = topic + "/compressed";   // 拼出压缩话题名
      auto tnat = nh_->get_topic_names_and_types();                // 获取系统所有话题及类型
      bool did_find_compressed_topic = false;
      for (auto topic_and_types : tnat) {
        if (topic_and_types.second.size() > 1) {
          // 明确避免"一个话题有多个类型"的情况（这种话题不处理）
          break;
        }
        auto & topic_name = topic_and_types.first;
        // 匹配话题名（兼容带/不带前导斜杠的写法）
        if(topic_name == compressed_topic_name || (topic_name.find("/") == 0 && topic_name.substr(1) == compressed_topic_name)){
          did_find_compressed_topic = true;   // 找到了压缩话题
          break;
        }
      }
      if (!did_find_compressed_topic)
      {
        RCLCPP_WARN(nh_->get_logger(), "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
        type = "mjpeg";   // 没找到 → 回退到 mjpeg
      }
    }

    // ---------- 创建流对象并启动 ----------
    // stream_types_[type] 是"工厂"，create_streamer 创建具体编码器的流对象
    boost::shared_ptr<ImageStreamer> streamer = stream_types_[type]->create_streamer(request, connection, nh_);
    streamer->start();   // 订阅 ROS 图像话题，开始接收图像
    // 把流加入活跃列表（加锁保护）
    boost::mutex::scoped_lock lock(subscriber_mutex_);
    image_subscribers_.push_back(streamer);
  }
  else
  {
    // 不支持的 type → 返回 404
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin,
                                                                                             end);
  }
  return true;
}

// ============================================================================
// handle_snapshot：处理 /snapshot 请求（单帧截图，推一帧 JPEG 就断开）
// URL 示例：/snapshot?topic=/camera/color/image_raw
// ============================================================================
bool WebVideoServer::handle_snapshot(const async_web_server_cpp::HttpRequest &request,
                                     async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                     const char* end)
{
  // 创建 JPEG 截图流对象（发一帧就自动关闭连接）
  boost::shared_ptr<ImageStreamer> streamer(new JpegSnapshotStreamer(request, connection, nh_));
  streamer->start();   // 订阅话题，等下一帧图像到来就截图发送

  boost::mutex::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(streamer);
  return true;
}

// ============================================================================
// handle_stream_viewer：处理 /stream_viewer 请求（返回 HTML 网页查看器）
// URL 示例：/stream_viewer?topic=/camera/color/image_raw&type=mjpeg
// 返回的网页里内嵌 <img src="/stream?...">，浏览器加载后自动持续请求视频流
// ============================================================================
bool WebVideoServer::handle_stream_viewer(const async_web_server_cpp::HttpRequest &request,
                                          async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                          const char* end)
{
  std::string type = request.get_query_param_value_or_default("type", __default_stream_type);
  if (stream_types_.find(type) != stream_types_.end())
  {
    std::string topic = request.get_query_param_value_or_default("topic", "");

    // 同样处理 ros_compressed 的压缩话题回退逻辑（与 handle_stream 相同）
    if (type == std::string("ros_compressed"))
    {
      std::string compressed_topic_name = topic + "/compressed";
      auto tnat = nh_->get_topic_names_and_types();
      bool did_find_compressed_topic = false;
      for (auto topic_and_types : tnat) {
        if (topic_and_types.second.size() > 1) {
          break;
        }
        auto & topic_name = topic_and_types.first;
        if(topic_name == compressed_topic_name || (topic_name.find("/") == 0 && topic_name.substr(1) == compressed_topic_name)){
          did_find_compressed_topic = true;
          break;
        }
      }
      if (!did_find_compressed_topic)
      {
        RCLCPP_WARN(nh_->get_logger(), "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
        type = "mjpeg";
      }
    }

    // 返回 HTML 响应头
    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
        "Server", "web_video_server").header("Content-type", "text/html;").write(connection);

    // 生成一个简单网页：标题 + 一个 <img> 标签指向 /stream
    std::stringstream ss;
    ss << "<html><head><title>" << topic << "</title></head><body>";
    ss << "<h1>" << topic << "</h1>";
    ss << stream_types_[type]->create_viewer(request);   // 生成 <img src="/stream?topic=..."> 标签
    ss << "</body></html>";
    connection->write(ss.str());
  }
  else
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin,
                                                                                             end);
  }
  return true;
}

// ============================================================================
// handle_list_streams：处理 / 请求（首页，列出所有 Image 话题）
// 遍历系统所有话题，筛出 Image 类型和 CameraInfo 类型，生成 HTML 列表
// ============================================================================
bool WebVideoServer::handle_list_streams(const async_web_server_cpp::HttpRequest &request,
                                         async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                         const char* end)
{
  std::vector<std::string> image_topics;        // 收集 Image 类型话题
  std::vector<std::string> camera_info_topics;  // 收集 CameraInfo 类型话题
  auto tnat = nh_->get_topic_names_and_types();  // 获取系统所有话题及类型
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // 明确避免一个话题对应多个类型的情况
      break;
    }
    auto & topic_name = topic_and_types.first;
    auto & topic_type = topic_and_types.second[0];  // 取第一个类型
    fprintf(stderr, "topic_type: %s\n", topic_type.c_str());
    // 按类型分类
    if (topic_type == "sensor_msgs/msg/Image")
    {
      image_topics.push_back(topic_name);
    }
    else if (topic_type == "sensor_msgs/msg/CameraInfo")
    {
      camera_info_topics.push_back(topic_name);
    }
  }

  // 返回 HTML 响应头（禁用缓存，保证列表实时刷新）
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Content-type", "text/html;").write(connection);

  // 输出页面标题
  connection->write("<html>"
                    "<head><title>ROS Image Topic List</title></head>"
                    "<body><h1>Available ROS Image Topics:</h1>");
  connection->write("<ul>");
  // ---------- 对有 camera_info 的话题：按相机分组显示 ----------
  // 例如 /camera/color/image_raw 和 /camera/color/camera_info，
  // 通过 camera_info 的 base_topic（/camera/color/）把属于同一相机的图像话题归到一起
  BOOST_FOREACH(std::string & camera_info_topic, camera_info_topics)
  {
    if (boost::algorithm::ends_with(camera_info_topic, "/camera_info"))
    {
      // 取出 base_topic（去掉末尾的 "camera_info"）
      std::string base_topic = camera_info_topic.substr(0, camera_info_topic.size() - strlen("camera_info"));
      connection->write("<li>");
      connection->write(base_topic);
      connection->write("<ul>");
      // 遍历图像话题，把属于这个相机（前缀匹配 base_topic）的挑出来生成链接
      std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
      for (; image_topic_itr != image_topics.end();)
      {
        if (boost::starts_with(*image_topic_itr, base_topic))
        {
          // 生成 stream_viewer 链接（网页查看）
          connection->write("<li><a href=\"/stream_viewer?topic=");
          connection->write(*image_topic_itr);
          connection->write("\">");
          connection->write(image_topic_itr->substr(base_topic.size()));  // 显示相对话题名
          connection->write("</a> (");
          // 生成 snapshot 链接（单帧截图）
          connection->write("<a href=\"/snapshot?topic=");
          connection->write(*image_topic_itr);
          connection->write("\">Snapshot</a>)");
          connection->write("</li>");

          image_topic_itr = image_topics.erase(image_topic_itr);  // 已处理，从列表移除
        }
        else
        {
          ++image_topic_itr;
        }
      }
      connection->write("</ul>");
    }
    connection->write("</li>");
  }
  connection->write("</ul>");
  // ---------- 输出剩余没有 camera_info 的图像话题 ----------
  connection->write("<ul>");
  std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
  for (; image_topic_itr != image_topics.end();) {
    connection->write("<li><a href=\"/stream_viewer?topic=");
    connection->write(*image_topic_itr);
    connection->write("\">");
    connection->write(*image_topic_itr);
    connection->write("</a> (");
    connection->write("<a href=\"/snapshot?topic=");
    connection->write(*image_topic_itr);
    connection->write("\">Snapshot</a>)");
    connection->write("</li>");

    image_topic_itr = image_topics.erase(image_topic_itr);
  }
  connection->write("</ul></body></html>");
  return true;
}

}

// ============================================================================
// main：程序入口
// 流程：初始化 ROS2 → 创建公共节点 + 私有节点 → 创建 WebVideoServer →
//       设置清理定时器 → 进入事件循环（spin）
// ============================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("web_video_server");          // 公共节点（订阅话题用）
  auto private_nh = std::make_shared<rclcpp::Node>("_web_video_server"); // 私有节点（读私有参数，节点名前加下划线）

  web_video_server::WebVideoServer server(nh, private_nh);
  server.setup_cleanup_inactive_streams();   // 启动 0.5s 清理定时器
  server.spin();                             // 启动 HTTP 服务器并阻塞运行

  return (0);
}
