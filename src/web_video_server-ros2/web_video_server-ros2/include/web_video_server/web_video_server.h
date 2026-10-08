#ifndef WEB_VIDEO_SERVER_H_
#define WEB_VIDEO_SERVER_H_

// ============================================================================
// 文件 : web_video_server.h
// 功能 : WebVideoServer 类声明——把 ROS 图像话题通过 HTTP 协议推流到浏览器
//
// 核心作用：
//   建立一个 HTTP 服务器，接收浏览器的请求，按请求的话题(topic)和编码格式(type)
//   订阅对应的 ROS 图像话题，把图像编码后通过 HTTP 持续推流给浏览器。
//
// 支持的 4 个 URL 路由（handle_xxx 分别处理）：
//   /                  → 列出所有可用的 Image 话题（首页）
//   /stream            → 视频流（持续推送，浏览器看到"视频"）
//   /snapshot          → 单帧截图（推一帧就断开）
//   /stream_viewer     → 返回一个 HTML 网页（内含 <img> 指向 /stream）
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "web_video_server/image_streamer.h"              // ImageStreamer 流基类 + 工厂基类
#include "async_web_server_cpp/http_server.hpp"           // 异步 HTTP 服务器（第三方库）
#include "async_web_server_cpp/http_request.hpp"          // HTTP 请求封装
#include "async_web_server_cpp/http_connection.hpp"       // HTTP 连接封装

namespace web_video_server
{

/**
 * @class WebVideoServer
 * @brief 核心类：HTTP 服务器 + ROS 图像推流管理
 */
class WebVideoServer
{
public:
  /**
   * @brief 构造函数
   * @param nh         公共节点（用于话题订阅等，节点名 "web_video_server"）
   * @param private_nh 私有节点（用于读取私有参数，节点名 "_web_video_server"）
   *
   * 构造时做的事：
   *   1. 读取参数（port 端口、address 地址、线程数、默认编码等）
   *   2. 注册 6 种编码器到 stream_types_（mjpeg/png/ros_compressed/vp8/h264/vp9）
   *   3. 注册 4 个 URL 路由到 handler_group_
   *   4. 创建 HttpServer 服务器对象
   */
  WebVideoServer(rclcpp::Node::SharedPtr &nh, rclcpp::Node::SharedPtr &private_nh);

  /**
   * @brief 析构函数
   */
  virtual ~WebVideoServer();

  /**
   * @brief 启动服务器并进入事件循环（阻塞直到关闭）
   *
   * 做的事：
   *   1. server_->run() 启动 HTTP 服务器
   *   2. 创建多线程执行器 spin（处理 ROS 回调，线程数 = ros_threads_）
   *   3. 若 publish_rate_>0，创建定时器定期 restreamFrames
   */
  void spin();

  /**
   * @brief 启动一个 0.5 秒周期的定时器，定期清理无连接的流
   */
  void setup_cleanup_inactive_streams();

  // ==================== 4 个 HTTP 路由处理函数 ====================
  // 每个函数的参数签名相同（HTTP 请求 + 连接 + 请求数据区间）
  // 当浏览器访问对应 URL 时，HTTP 服务器会调用对应的 handle_xxx

  /**
   * @brief 处理 /stream 请求：视频流（核心！持续推流）
   * @param type  URL 参数（mjpeg/png/...），决定用哪种编码器
   * @param topic URL 参数（如 /camera/color/image_raw），决定订阅哪个话题
   * 流程：创建对应编码器的 Streamer → 订阅 ROS 话题 → 持续推送图像
   */
  bool handle_stream(const async_web_server_cpp::HttpRequest &request,
                     async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

  /**
   * @brief 处理 /stream_viewer 请求：返回一个 HTML 网页查看器
   * 网页里内嵌 <img src="/stream?..."> 标签，浏览器加载后自动持续请求视频流
   */
  bool handle_stream_viewer(const async_web_server_cpp::HttpRequest &request,
                            async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

  /**
   * @brief 处理 /snapshot 请求：单帧截图（推一帧 JPEG 就断开连接）
   * 适合"刷新一次看一帧"的场景，不适合连续视频
   */
  bool handle_snapshot(const async_web_server_cpp::HttpRequest &request,
                       async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

  /**
   * @brief 处理 / 请求：首页，列出当前系统所有 Image 话题
   * 并给每个话题生成 /stream_viewer 和 /snapshot 的链接
   */
  bool handle_list_streams(const async_web_server_cpp::HttpRequest &request,
                           async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

private:
  /**
   * @brief 把"最新一帧"重新推送给所有活跃的流（用于控制推流频率）
   * @param max_age 超过该时间（秒）没更新的帧才重新推送
   */
  void restreamFrames(double max_age);

  /**
   * @brief 清理没有浏览器连接的流（inactive），并取消对应的 ROS 订阅
   * 作用：浏览器关闭后及时释放订阅和资源，避免浪费带宽/CPU
   */
  void cleanup_inactive_streams();

  // ==================== 成员变量 ====================
  rclcpp::Node::SharedPtr nh_;                       // 公共节点（话题订阅用）
  rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr cleanup_timer_;  // 清理定时器
  int ros_threads_;                                  // ROS 回调线程数
  double publish_rate_;                              // 推流频率（-1 表示不限速）
  int port_;                                         // HTTP 服务端口（默认 8080）
  std::string address_;                              // 监听地址（默认 0.0.0.0）
  boost::shared_ptr<async_web_server_cpp::HttpServer> server_;  // 异步 HTTP 服务器实例
  async_web_server_cpp::HttpRequestHandlerGroup handler_group_; // 路由表（URL → 处理函数）

  std::vector<boost::shared_ptr<ImageStreamer> > image_subscribers_;  // 所有活跃的流（订阅者）
  std::map<std::string, boost::shared_ptr<ImageStreamerType> > stream_types_;  // 编码器注册表（type名 → 工厂）
  boost::mutex subscriber_mutex_;                     // 保护 image_subscribers_ 的互斥锁
};

}

#endif
