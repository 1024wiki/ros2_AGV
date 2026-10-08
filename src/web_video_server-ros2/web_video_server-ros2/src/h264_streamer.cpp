#include "web_video_server/h264_streamer.h"

namespace web_video_server
{

H264Streamer::H264Streamer(const async_web_server_cpp::HttpRequest& request,
                         async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
    LibavStreamer(request, connection, nh, "mp4", "libx264", "video/mp4")
{
  /* possible quality presets:
   * ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow, placebo
   * no latency improvements observed with ultrafast instead of medium
   */
  preset_ = request.get_query_param_value_or_default("preset", "ultrafast");
}

H264Streamer::~H264Streamer()
{
}

// ============================================================================
// initializeEncoder：重写基类留白虚函数，配置 x264 编码器私有参数与 MP4 容器选项
//
// 核心目标：
//   1. 调优 x264 编码器达到“零延迟实时推流”（禁用 B 帧、禁用前瞻缓存、极速编码）
//   2. 启用 fMP4（分片 MP4）格式，使 MP4 容器支持流式输出并在浏览器 <video> 实时秒开
// ============================================================================
void H264Streamer::initializeEncoder()
{
  // 1. 设置编码速度/压缩率预设（默认 "ultrafast"）：
  //    关闭耗时的运动搜索与计算，以最低的 CPU 占用换取毫秒级吞吐，保证机器人上位机低算力开销
  av_opt_set(codec_context_->priv_data, "preset", preset_.c_str(), 0);

  // 2. 启用零延迟调优（★低延迟最关键参数）：
  //    - 彻底关闭 B 帧（无双向参考帧，无需等待未来帧，消除乱序延迟）
  //    - 关闭多线程前向预读（lookahead=0，实现“吃进一帧、立即编码输出一帧”，1 in, 1 out）
  av_opt_set(codec_context_->priv_data, "tune", "zerolatency", 0);

  // 3. 恒定速率因子（CRF=20，画质控制）：
  //    范围 0~51，数值越小画质越好。20 属于肉眼几乎无损的高清画质，兼顾清晰度与低码率
  av_opt_set_int(codec_context_->priv_data, "crf", 20, 0);

  // 4. VBV 缓冲区大小（bufsize=100）：
  //    极小缓冲区限制码率突发，强制让码率在极窄窗口内平顺输出，防止因某一帧数据过大造成网络卡顿
  av_opt_set_int(codec_context_->priv_data, "bufsize", 100, 0);

  // 5. 最大关键帧间隔（GOP 大小，keyint=30）：
  //    每 30 帧必须强制生成一个 IDR 关键帧（30fps 下为 1 秒一帧）；
  //    确保新打开网页的客户端最多等 1 秒即可秒开画面，且网络丢包花屏后 1 秒内能立即恢复
  av_opt_set_int(codec_context_->priv_data, "keyint", 30, 0);

  // 6. 显式设置最小关键帧间隔（g=1），配合 keyint 控制关键帧生成节奏
  av_opt_set_int(codec_context_->priv_data, "g", 1, 0);

  // container format options
  // ---- 7. 配置 MP4 容器封装选项（将离线文件格式转变为实时直播流格式） ----
  if (!strcmp(format_context_->oformat->name, "mp4")) {
    // set up mp4 for streaming (instead of seekable file output)
    // +frag_keyframe : 开启 fMP4（分片 MP4）模式，每个关键帧生成一个独立的 moof+mdat 迷你片段，边推边播
    // +empty_moov    : 文件最前端生成空索引头，浏览器一连上来就能初始化解码器，无需等待整个视频录制完成
    // +faststart     : 将必要的基础元数据前置到流的最开头，保证客户端秒开
    av_dict_set(&opt_, "movflags", "+frag_keyframe+empty_moov+faststart", 0);
  }
}

H264StreamerType::H264StreamerType() :
    LibavStreamerType("mp4", "libx264", "video/mp4")
{
}

boost::shared_ptr<ImageStreamer> H264StreamerType::create_streamer(const async_web_server_cpp::HttpRequest& request,
                                                                  async_web_server_cpp::HttpConnectionPtr connection,
                                                                  rclcpp::Node::SharedPtr nh)
{
  return boost::shared_ptr<ImageStreamer>(new H264Streamer(request, connection, nh));
}

}
