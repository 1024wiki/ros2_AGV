// ============================================================================
// 文件 : libav_streamer.cpp
// 功能 : FFmpeg 产品基类实现——真正的视频编码（帧间预测压缩）
//
// 本文件实现 LibavStreamer（第二层模板骨架）：首帧建 FFmpeg 管线，
// 逐帧 BGR→YUV→编码→写容器；以及 LibavStreamerType（参数化工厂）。
// H264/VP8/VP9 只重写 initializeEncoder() 几行私有参数，其余全复用此处。
// 只加注释，不改任何源代码逻辑。
// ============================================================================
#include "web_video_server/libav_streamer.h" // 本类声明：LibavStreamer/LibavStreamerType，含 AVFormat/AVCodec 等成员
#include "async_web_server_cpp/http_reply.hpp" // HTTP 响应构造器：出错回 500、成功发视频响应头

/*https://stackoverflow.com/questions/46884682/error-in-building-opencv-with-ffmpeg*/
// 兼容宏：老版本 FFmpeg 头文件缺失 AV_CODEC_FLAG_GLOBAL_HEADER 时手动定义
// AV_CODEC_FLAG_GLOBAL_HEADER 含义：把 SPS/PPS 等全局头只写在容器头而不重复写进每个关键帧
// (1 << 22) 为该标志在旧版 libavcodec 中的位位置；新版已在 avcodec.h 定义，此处重复定义值相同无害
#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER // 别名：兼容旧代码 CODEC_FLAG_* 写法

namespace web_video_server
{

// ----------------------------------------------------------------------------
// ffmpeg 线程锁管理器：把 FFmpeg 内部的多线程锁回调接到 boost::mutex 上
// FFmpeg 编码器内部会多线程访问共享结构，需向 av_lockmgr_register 注册此函数；
// op 四种操作：CREATE 新建锁 / OBTAIN 加锁 / RELEASE 解锁 / DESTROY 销毁锁
// ----------------------------------------------------------------------------
static int ffmpeg_boost_mutex_lock_manager(void **mutex, enum AVLockOp op)
{
  if (NULL == mutex)
    return -1;

  switch (op)
  {
    case AV_LOCK_CREATE:
    {
      *mutex = NULL;
      boost::mutex *m = new boost::mutex();
      *mutex = static_cast<void *>(m);
      break;
    }
    case AV_LOCK_OBTAIN:
    {
      boost::mutex *m = static_cast<boost::mutex *>(*mutex);
      m->lock();
      break;
    }
    case AV_LOCK_RELEASE:
    {
      boost::mutex *m = static_cast<boost::mutex *>(*mutex);
      m->unlock();
      break;
    }
    case AV_LOCK_DESTROY:
    {
      boost::mutex *m = static_cast<boost::mutex *>(*mutex);
      m->lock();
      m->unlock();
      delete m;
      break;
    }
    default:
      break;
  }
  return 0;
}

// ----------------------------------------------------------------------------
// LibavStreamer 构造：先调中间层骨架构造，再保存三配方 + 解析编码 URL 参数
// format_name/codec_name/content_type：由工厂传入（如 mp4/libx264/video/mp4）
// FFmpeg 指针成员全部置 0（管线在首帧 initialize() 里建，此处不建）
// bitrate/qmin/qmax/gop：URL 可覆盖的质量与码率参数
// 最后注册线程锁管理器 + 注册全部编解码器（老 FFmpeg 必需）
// ----------------------------------------------------------------------------
LibavStreamer::LibavStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh,
                             const std::string &format_name, const std::string &codec_name,
                             const std::string &content_type) :
    ImageTransportImageStreamer(request, connection, nh), output_format_(0), format_context_(0), codec_(0), codec_context_(0), video_stream_(
        0), frame_(0), sws_context_(0), first_image_timestamp_(0), format_name_(
        format_name), codec_name_(codec_name), content_type_(content_type), opt_(0), io_buffer_(0)
{

  // ---- 解析 URL 编码质量参数（均可在浏览器 ?bitrate=&qmin=&qmax=&gop= 覆盖） ----
  // bitrate_：目标码率 bps，默认 100000(100kbps)，越大约清晰但越占带宽
  // qmin_/qmax_：量化参数范围 0~51，越小越清晰；默认 10~42兼顾清晰与体积
  // gop_：Group of Pictures，多少帧一个 I 帧，默认 250；越小恢复越快但体积越大
  bitrate_ = request.get_query_param_value_or_default<int>("bitrate", 100000);
  qmin_ = request.get_query_param_value_or_default<int>("qmin", 10);
  qmax_ = request.get_query_param_value_or_default<int>("qmax", 42);
  gop_ = request.get_query_param_value_or_default<int>("gop", 250);

  // 注册 FFmpeg 线程锁回调（多浏览器并发编码时保护 FFmpeg 全局表）
  // + 注册所有 muxer/encoder（老 FFmpeg 必需，新版 av_register_all 已为空操作，保留为兼容）
  av_lockmgr_register(&ffmpeg_boost_mutex_lock_manager);
  av_register_all();
}

// ----------------------------------------------------------------------------
// 析构：按申请逆序释放 FFmpeg 资源（兼容老版本 API 分支）
// ----------------------------------------------------------------------------
LibavStreamer::~LibavStreamer()
{
  // 关闭编码器上下文
  if (codec_context_)
    avcodec_close(codec_context_);
  if (frame_)
  {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
    av_free(frame_);
    frame_ = NULL;
#else
    av_frame_free(&frame_);
#endif
  }
  if (io_buffer_)
    delete io_buffer_;
  if (format_context_) {
    if (format_context_->pb)
      av_free(format_context_->pb);
    avformat_free_context(format_context_);
  }
  if (sws_context_)
    sws_freeContext(sws_context_);
}

// ----------------------------------------------------------------------------
// FFmpeg 自定义 IO 回调：编码器每编码出一个packet并在av_write_frame写入封装上下文中就调一次，直接写入 socket 进行发送
// opaque：构造时传入的 &connection_；buffer/buffer_size：本次产出的数据
// 返回 0 表示成功
// ----------------------------------------------------------------------------
// output callback for ffmpeg IO context
static int dispatch_output_packet(void* opaque, uint8_t* buffer, int buffer_size)
{
  // 1. 取出我们在上面绑定的 HTTP 连接
  async_web_server_cpp::HttpConnectionPtr connection = *((async_web_server_cpp::HttpConnectionPtr*) opaque);
  
  // 2. 把 FFmpeg 封装好的这几个字节（比如 fMP4 的一个分片 moof/mdat）转成数组
  std::vector<uint8_t> encoded_frame;
  encoded_frame.assign(buffer, buffer + buffer_size);
  
  // 3. ★ 直接通过 Socket 发送给远端浏览器！
  connection->write_and_clear(encoded_frame);
  return 0;
}

// ----------------------------------------------------------------------------
// initialize：首帧调用一次，建好整条 FFmpeg 推流管线（模板第二层骨架核心）
// 步骤：分配格式上下文→猜容器→绑自定义 IO→找编码器→建视频流→填参数→
//       调子类 initializeEncoder()→开编码器→分配帧→发 HTTP 头→写容器头
// 任何一步失败：先回 500 再抛异常（imageCallback 捕获后置失效）
// ----------------------------------------------------------------------------
void LibavStreamer::initialize(const cv::Mat &img)
{
  // Load format
  // 步骤 1：分配输出格式上下文（装整个容器状态的大结构体）
  format_context_ = avformat_alloc_context();
  if (!format_context_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error allocating ffmpeg format context");
  }
  output_format_ = av_guess_format(format_name_.c_str(), NULL, NULL);
  if (!output_format_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error looking up output format");
  }
  format_context_->oformat = output_format_;

  // Set up custom IO callback.
  // 步骤 2：绑自定义 IO——FFmpeg 输出不写文件，经 dispatch_output_packet 直写 socket
  size_t io_buffer_size = 3 * 1024;    // 3M seen elsewhere and adjudged good
  io_buffer_ = new unsigned char[io_buffer_size];
  AVIOContext* io_ctx = avio_alloc_context(io_buffer_, io_buffer_size, AVIO_FLAG_WRITE, &connection_, NULL, dispatch_output_packet, NULL);
  if (!io_ctx)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error setting up IO context");
  }
  io_ctx->seekable = 0;                       // no seeking, it's a stream
  format_context_->pb = io_ctx;
  output_format_->flags |= AVFMT_FLAG_CUSTOM_IO;
  output_format_->flags |= AVFMT_NOFILE;

  // Load codec
  // 步骤 3：找编码器 + 建视频流（模板第二层骨架核心前置）
  // codec_name_ 由工厂传入：H264 传 libx264，VP8 传 libvpx，VP9 传 libvpx-vp9；为空则用容器默认编码
  if (codec_name_.empty()) // use default codec if none specified
    codec_ = avcodec_find_encoder(output_format_->video_codec); // 按容器默认视频编码 ID 查找
  else
    codec_ = avcodec_find_encoder_by_name(codec_name_.c_str()); // 按名查找（如 libx264）
  if (!codec_)
  {
    // 找不到编码器（FFmpeg 未编译该编码器）：先回 HTTP 500，再抛异常终止本流
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error looking up codec");
  }
  // 在格式上下文中新建一条视频流（单视频流场景，index 固定为 0）
  video_stream_ = avformat_new_stream(format_context_, codec_);
  if (!video_stream_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error creating video stream");
  }
  codec_context_ = video_stream_->codec; // 老 API：编码上下文挂在视频流上（新 API 已独立，此处为兼容写法）

  // Set options
  // 先用编码器默认值填充上下文，避免野值；再逐项覆盖本项目所需字段
  avcodec_get_context_defaults3(codec_context_, codec_);

  codec_context_->codec_id = codec_->id; // 编码 ID 与查找到的编码器一致（如 AV_CODEC_ID_H264）
  codec_context_->bit_rate = bitrate_;    // 目标码率：控体积与清晰度平衡

  codec_context_->width = output_width_;  // 编码宽：取 URL 缩放后尺寸，与 Mat 一致
  codec_context_->height = output_height_; // 编码高：同上
  codec_context_->delay = 0;              // 编码延迟 0：不缓存，低延迟直播必需

  // 容器时间基 1/1000（毫秒）：pts 以毫秒为单位递增，便于与 ROS 时间戳换算
  video_stream_->time_base.num = 1;
  video_stream_->time_base.den = 1000;

  // 编码器时间基 1/1：每帧 pts 加 1 的节奏由上层按秒换算填入
  codec_context_->time_base.num = 1;
  codec_context_->time_base.den = 1;
  codec_context_->gop_size = gop_;              // I 帧间隔：URL gop 参数
  codec_context_->pix_fmt = AV_PIX_FMT_YUV420P; // 编码像素格式：YUV420P，最通用
  codec_context_->max_b_frames = 0;             // 禁 B 帧：B 帧需依赖未来帧，会引入乱序延迟

  // Quality settings
  codec_context_->qmin = qmin_; // 最小量化：画质上限
  codec_context_->qmax = qmax_; // 最大量化：体积下限

  initializeEncoder(); // ★ 第二层留白：H264 填 preset/tune/crf，VP8/VP9 填各自实时参数

  // Some formats want stream headers to be separate
  if (format_context_->oformat->flags & AVFMT_GLOBALHEADER)
    codec_context_->flags |= CODEC_FLAG_GLOBAL_HEADER;

  // Open Codec
  if (avcodec_open2(codec_context_, codec_, NULL) < 0)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Could not open video codec");
  }

  // Allocate frame buffers
  // 分配编码用帧缓冲：按输出宽高×YUV420P 分配内存；版本宏兼容新老 FFmpeg
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
  frame_ = avcodec_alloc_frame();
#else
  frame_ = av_frame_alloc();
#endif
  av_image_alloc(frame_->data, frame_->linesize, output_width_, output_height_,
          codec_context_->pix_fmt, 1);

  frame_->width = output_width_;   // 帧宽与编码上下文一致
  frame_->height = output_height_; // 帧高与编码上下文一致
  frame_->format = codec_context_->pix_fmt; // 帧格式 YUV420P
  output_format_->flags |= AVFMT_NOFILE; // 无文件输出：纯流式

  // Generate header
  std::vector<uint8_t> header_buffer; // 预留：容器头缓冲（本实现直接经 IO 回调发出，未本地缓存）
  std::size_t header_size;
  uint8_t *header_raw_buffer;
  // define meta data
  // 写入容器元数据：作者固定，标题为话题名，浏览器调试可辨识流来源
  av_dict_set(&format_context_->metadata, "author", "ROS web_video_server", 0);
  av_dict_set(&format_context_->metadata, "title", topic_.c_str(), 0);

  // Send response headers
  // 发 HTTP 响应头：200 + 禁缓存 + Content-type 为 video/mp4 或 video/webm + 允许跨域
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Expires", "0").header("Max-Age", "0").header("Trailer", "Expires").header(
      "Content-type", content_type_).header("Access-Control-Allow-Origin", "*").write(connection_);

  // Send video stream header
  // 写容器头（mp4 moov / webm 头）：经自定义 IO 直接发往 socket；失败回 500 并抛异常
  if (avformat_write_header(format_context_, &opt_) < 0)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error openning dynamic buffer");
  }
}

void LibavStreamer::initializeEncoder()
{
  // 基类空实现：Mjpeg/Png 无需编码器私有参数；H264/VP8/VP9 各自重写填 av_opt_set 参数
}

// ============================================================================
// sendImage：逐帧视频编码与网络推流（整个 FFmpeg 视频管线的每帧驱动核心）
//
// 核心步骤：
//   1. 线程互斥加锁，防多线程重入破坏 FFmpeg 编码器上下文
//   2. 首帧时间戳对齐，建立基准时间
//   3. 将 OpenCV BGR24 格式 Mat 数据包装进 FFmpeg 原生 AVFrame
//   4. 颜色空间转换：通过 sws_scale 将 BGR24 像素转为 YUV420P（libx264 编码器要求）
//   5. 核心编码：调用 FFmpeg 编码 API（兼容老版本与现代 send/receive 异步模型）
//   6. 时间戳计算：按相对时间折算 PTS 并乘以 0.95 追赶播放以压制端到端网络累积延迟
//   7. 容器封装并直发：av_write_frame 触发内存 AVIO 回调，将 fMP4/WebM 切片直推 Socket
//   8. 资源释放与清理
// ============================================================================
void LibavStreamer::sendImage(const cv::Mat &img, const rclcpp::Time &time)
{
  // 1. 加互斥锁：同一连接在多线程执行器环境下串行执行编码，确保 FFmpeg 上下文线程安全
  boost::mutex::scoped_lock lock(encode_mutex_);

  // 2. 记录首帧 ROS 物理时间戳：后续所有帧的相对显示时间戳（PTS）均以此为 0 点基准计算
  if (0 == first_image_timestamp_.nanoseconds())
  {
    first_image_timestamp_ = time;
  }
  std::vector<uint8_t> encoded_frame;

  // 区分老旧版本 libavutil 与现代版本的像素格式枚举名定义
#if (LIBAVUTIL_VERSION_MAJOR < 53)
  PixelFormat input_coding_format = PIX_FMT_BGR24;
#else
  AVPixelFormat input_coding_format = AV_PIX_FMT_BGR24;
#endif

  // 3. 将 OpenCV Mat 的裸内存指针包装为 FFmpeg 的输入图像结构（兼容旧版 AVPicture 与新版 AVFrame）
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
  AVPicture *raw_frame = new AVPicture;
  avpicture_fill(raw_frame, img.data, input_coding_format, output_width_, output_height_);
#else
  AVFrame *raw_frame = av_frame_alloc();
  // 将 img.data（BGR 原始像素数组）映射关联到 raw_frame->data 和行对齐宽度 linesize，零内存拷贝包装
  av_image_fill_arrays(raw_frame->data, raw_frame->linesize,
                       img.data, input_coding_format, output_width_, output_height_, 1);
#endif

  // 4. 颜色空间转换器初始化（懒加载单例复用）：
  //    输入格式：BGR24（OpenCV 格式），输出格式：YUV420P（H.264/VP8 编码通用格式）
  //    采用 SWS_BICUBIC 双三次插值算法保证转换后的画面平滑清晰
  if (!sws_context_)
  {
    static int sws_flags = SWS_BICUBIC;
    sws_context_ = sws_getContext(output_width_, output_height_, input_coding_format, 
                                  output_width_, output_height_, codec_context_->pix_fmt, 
                                  sws_flags, NULL, NULL, NULL);
    if (!sws_context_)
    {
      throw std::runtime_error("Could not initialize the conversion context");
    }
  }

  // 5. 执行色彩空间转换：从 raw_frame(BGR24) 缩放转换到 frame_(YUV420P) 准备送入编码器
  int ret = sws_scale(sws_context_,
          (const uint8_t * const *)raw_frame->data, raw_frame->linesize, 0,
          output_height_, frame_->data, frame_->linesize);

  // 临时包装的 raw_frame 结构体使命完成，立即释放（底层的 img 图像内存由 OpenCV 管理，不受影响）
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
  delete raw_frame;
#else
  av_frame_free(&raw_frame);
#endif

  // 6. 初始化编码产物数据包 AVPacket
  AVPacket pkt;
  int got_packet;
  av_init_packet(&pkt);

  // ---- 7. 核心编码步骤（代码利用预编译宏兼容了历史上三代 FFmpeg 编码 API） ----
#if (LIBAVCODEC_VERSION_MAJOR < 54)
  // [第一代老 API]: 手动分配固定大小内存进行视频编码
  int buf_size = 6 * output_width_ * output_height_;
  pkt.data = (uint8_t*)av_malloc(buf_size);
  pkt.size = avcodec_encode_video(codec_context_, pkt.data, buf_size, frame_);
  got_packet = pkt.size > 0;
#elif (LIBAVCODEC_VERSION_MAJOR < 57)
  // [第二代 API]: avcodec_encode_video2，编码器自动分配包内存，通过 got_packet 标记是否吐出有效压缩包
  pkt.data = NULL; // packet data will be allocated by the encoder
  pkt.size = 0;
  if (avcodec_encode_video2(codec_context_, &pkt, frame_, &got_packet) < 0)
  {
     throw std::runtime_error("Error encoding video frame");
  }
#else
  // [第三代现代标准 API]: 异步双向解耦模型（send_frame 喂入未压缩像素，receive_packet 拉取压缩码流）
  pkt.data = NULL; // packet data will be allocated by the encoder
  pkt.size = 0;
  // 将 YUV420P 像素帧推入编码器管线
  if (avcodec_send_frame(codec_context_, frame_) < 0)
  {
    throw std::runtime_error("Error encoding video frame");
  }
  // 从编码器中收取编码完成的压缩数据包（若尚未完成则返回错误或等待）
  if (avcodec_receive_packet(codec_context_, &pkt) < 0)
  {
    throw std::runtime_error("Error retrieving encoded packet");
  }
  got_packet = 1; // 现代 API 若 receive_packet 成功返回 0，表明必定收到了完整有效的数据包
#endif

  // ---- 8. 如果成功产出了编码压缩数据包（例如产生了一个 H.264 NALU） ----
  if (got_packet)
  {
    std::size_t size;
    uint8_t *output_buf;

    // 计算当前帧相对于首帧的真实物理流逝时间（秒）
    double seconds = (time - first_image_timestamp_).seconds();

    // ★ 核心低延迟调优技巧：
    // 将秒数映射为容器时间基单位（毫秒，1/1000），并乘以 0.95 进行“微加速追赶”！
    // 原理：让 PTS 的时间戳推进得稍微比物理时钟快 5%，迫使前端播放器（如浏览器 <video>）
    //       稍微加快播放速率，以此主动消耗网络排队缓冲区，防止端到端延迟随着推流时间累积！
    pkt.pts = (int64_t)(seconds / av_q2d(video_stream_->time_base) * 0.95);
    if (pkt.pts <= 0)
      pkt.pts = 1;             // 保证 PTS 严格大于 0，防止被某些封装器判定为非法时间戳
    pkt.dts = AV_NOPTS_VALUE;  // 由于我们在前面完全禁用了 B 帧，显示顺序即解码顺序，DTS 设为无效即可

    // 如果当前帧是 IDR 关键帧，透传关键帧标志位（告知解复用器和浏览器此处可独立秒开）
    if (pkt.flags&AV_PKT_FLAG_KEY)
      pkt.flags |= AV_PKT_FLAG_KEY;

    pkt.stream_index = video_stream_->index; // 关联到唯一的视频流索引（通常为 0）

    // 9. 将数据包写入容器格式（MP4 / WebM）：
    //    由于我们在前面开启了 +frag_keyframe，遇到关键帧时会自动生成 moof+mdat 迷你切片，
    //    此函数内部会自动触发我们注册的 AVIO 回调函数 dispatch_output_packet，
    //    将组装好的 fMP4 字节数据直接写入 TCP Socket 发往远端浏览器！
    if (av_write_frame(format_context_, &pkt))
    {
      throw std::runtime_error("Error when writing frame");
    }
  }
  else
  {
    encoded_frame.clear();
  }

  // ---- 10. 释放本次产生的 AVPacket 资源，防止内存泄露 ----
#if LIBAVCODEC_VERSION_INT < 54
  av_free(pkt.data);
#endif

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
  av_free_packet(&pkt);
#else
  av_packet_unref(&pkt); // 现代 API：释放数据包内部引用计数的缓冲内存
#endif

  // 清空本轮未用的临时缓冲区
  connection_->write_and_clear(encoded_frame);
}

LibavStreamerType::LibavStreamerType(const std::string &format_name, const std::string &codec_name,
                                     const std::string &content_type) :
    format_name_(format_name), codec_name_(codec_name), content_type_(content_type)
{
}

boost::shared_ptr<ImageStreamer> LibavStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                                    async_web_server_cpp::HttpConnectionPtr connection,
                                                                    rclcpp::Node::SharedPtr nh)
{
  return boost::shared_ptr<ImageStreamer>(
      new LibavStreamer(request, connection, nh, format_name_, codec_name_, content_type_));
}

std::string LibavStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<video src=\"/stream?";
  ss << request.query;
  ss << "\" autoplay=\"true\" preload=\"none\"></video>";
  return ss.str();
}

}
