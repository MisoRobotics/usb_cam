/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#define __STDC_CONSTANT_MACROS
#include <usb_cam/usb_cam.h>
#include <usb_cam/usb_cam_config.h>
#include <cstdio>
#include <stdlib.h>
#include <string>
#include <cassert>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <array>

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/fill_image.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

namespace usb_cam {

static void errno_exit(const char* error, const char* location)
{
  if (ros::ok()) {
    ROS_ERROR("%s error %d (%s) in usb_cam::%s", error, errno, strerror(errno), location);
  } else {
    fprintf(stderr, "%s error %d (%s) in usb_cam::%s\n", error, errno, strerror(errno), location);
  }
  exit(EXIT_FAILURE);
}

static void error_exit(const char * s)
{
  if (ros::ok()) {
    ROS_ERROR("[usb_cam error] %s", s);
  } else {
    fprintf(stderr, "[usb_cam error] %s\n", s);
  }
  exit(EXIT_FAILURE);
}

static int xioctl(int fd, int request, void * arg)
{
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

const unsigned char uchar_clipping_table[] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -128 - -121
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -120 - -113
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -112 - -105
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -104 -  -97
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -96 -  -89
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -88 -  -81
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -80 -  -73
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -72 -  -65
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -64 -  -57
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -56 -  -49
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -48 -  -41
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -40 -  -33
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -32 -  -25
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -24 -  -17
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -16 -   -9
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //   -8 -   -1
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88,
    89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
    114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136,
    137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
    160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
    183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205,
    206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228,
    229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251,
    252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 256-263
    255, 255, 255, 255, 255, 255, 255, 255, // 264-271
    255, 255, 255, 255, 255, 255, 255, 255, // 272-279
    255, 255, 255, 255, 255, 255, 255, 255, // 280-287
    255, 255, 255, 255, 255, 255, 255, 255, // 288-295
    255, 255, 255, 255, 255, 255, 255, 255, // 296-303
    255, 255, 255, 255, 255, 255, 255, 255, // 304-311
    255, 255, 255, 255, 255, 255, 255, 255, // 312-319
    255, 255, 255, 255, 255, 255, 255, 255, // 320-327
    255, 255, 255, 255, 255, 255, 255, 255, // 328-335
    255, 255, 255, 255, 255, 255, 255, 255, // 336-343
    255, 255, 255, 255, 255, 255, 255, 255, // 344-351
    255, 255, 255, 255, 255, 255, 255, 255, // 352-359
    255, 255, 255, 255, 255, 255, 255, 255, // 360-367
    255, 255, 255, 255, 255, 255, 255, 255, // 368-375
    255, 255, 255, 255, 255, 255, 255, 255, // 376-383
    };
const int clipping_table_offset = 128;

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
static unsigned char CLIPVALUE(int val)
{
  // Old method (if)
  /*   val = val < 0 ? 0 : val; */
  /*   return val > 255 ? 255 : val; */

  // New method (array)
  return uchar_clipping_table[val + clipping_table_offset];
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
static void YUV2RGB(const unsigned char y, const unsigned char u, const unsigned char v, unsigned char* r,
                    unsigned char* g, unsigned char* b)
{
  const int y2 = (int)y;
  const int u2 = (int)u - 128;
  const int v2 = (int)v - 128;
  //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);
  //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

void uyvy2rgb(char *YUV, char *RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
  {
    u = (unsigned char)YUV[i + 0];
    y0 = (unsigned char)YUV[i + 1];
    v = (unsigned char)YUV[i + 2];
    y1 = (unsigned char)YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

static void mono102mono8(char *RAW, char *MONO, int NumPixels)
{
  int i, j;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1)
  {
    //first byte is low byte, second byte is high byte; smash together and convert to 8-bit
    MONO[j] = (unsigned char)(((RAW[i + 0] >> 2) & 0x3F) | ((RAW[i + 1] << 6) & 0xC0));
  }
}

static void yuyv2rgb(char *YUV, char *RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
  {
    y0 = (unsigned char)YUV[i + 0];
    u = (unsigned char)YUV[i + 1];
    y1 = (unsigned char)YUV[i + 2];
    v = (unsigned char)YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

void rgb242rgb(char *YUV, char *RGB, int NumPixels)
{
  memcpy(RGB, YUV, NumPixels * 3);
}

UsbCam::UsbCam()
  : io_(IO_METHOD_MMAP), fd_(-1), buffers_(NULL), n_buffers_(0), avframe_camera_(NULL),
    avframe_rgb_(NULL), avcodec_(NULL), avoptions_(NULL), avcodec_context_(NULL),
    avframe_camera_size_(0), avframe_rgb_size_(0), video_sws_(NULL), image_(NULL),
    is_capturing_(false), buffers_allocated_(false) {
}
UsbCam::~UsbCam()
{
  shutdown();
}

int UsbCam::init_mjpeg_decoder(int image_width, int image_height)
{
  avcodec_register_all();

  avcodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!avcodec_)
  {
    ROS_ERROR("Could not find MJPEG decoder");
    return 0;
  }

  avcodec_context_ = avcodec_alloc_context3(avcodec_);
#if LIBAVCODEC_VERSION_MAJOR < 55
  avframe_camera_ = avcodec_alloc_frame();
  avframe_rgb_ = avcodec_alloc_frame();
#else
  avframe_camera_ = av_frame_alloc();
  avframe_rgb_ = av_frame_alloc();
#endif

  avpicture_alloc((AVPicture *)avframe_rgb_, PIX_FMT_RGB24, image_width, image_height);

  avcodec_context_->codec_id = AV_CODEC_ID_MJPEG;
  avcodec_context_->width = image_width;
  avcodec_context_->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
  avcodec_context_->pix_fmt = PIX_FMT_YUV422P;
  avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif

  avframe_camera_size_ = avpicture_get_size(PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ = avpicture_get_size(PIX_FMT_RGB24, image_width, image_height);

  /* open it */
  if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0)
  {
    ROS_ERROR("Could not open MJPEG Decoder");
    return 0;
  }
  return 1;
}

AVPixelFormat UsbCam::fix_pixel_format(AVPixelFormat fmt)
{
  switch (avcodec_context_->pix_fmt)
  {
    case AV_PIX_FMT_YUVJ411P: return AV_PIX_FMT_YUV411P;
    case AV_PIX_FMT_YUVJ420P: return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P: return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ440P: return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ444P: return AV_PIX_FMT_YUV444P;
    default: return fmt;
  }
}

void UsbCam::mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels)
{
  int got_picture;

  memset(RGB, 0, avframe_rgb_size_);

#if LIBAVCODEC_VERSION_MAJOR > 52
  int decoded_len;
  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.size = len;
  avpkt.data = (unsigned char*)MJPEG;
  decoded_len = avcodec_decode_video2(avcodec_context_, avframe_camera_, &got_picture, &avpkt);

  if (decoded_len < 0)
  {
    ROS_ERROR("Error while decoding frame.");
    return;
  }
#else
  avcodec_decode_video(avcodec_context_, avframe_camera_, &got_picture, (uint8_t *) MJPEG, len);
#endif

  // avcodec_decode_video mangles certain pixel formats in the codec context; fix that here:
  avcodec_context_->pix_fmt = fix_pixel_format(avcodec_context_->pix_fmt);

  if (!got_picture)
  {
    ROS_ERROR("Webcam: expected picture but didn't get it...");
    return;
  }

  int xsize = avcodec_context_->width;
  int ysize = avcodec_context_->height;
  int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
  if (pic_size != avframe_camera_size_)
  {
    ROS_ERROR("outbuf size mismatch.  pic_size: %d bufsize: %d", pic_size, avframe_camera_size_);
    return;
  }

  video_sws_ = sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize, PIX_FMT_RGB24, SWS_BILINEAR, NULL,
			      NULL,  NULL);
  sws_scale(video_sws_, avframe_camera_->data, avframe_camera_->linesize, 0, ysize, avframe_rgb_->data,
            avframe_rgb_->linesize);
  sws_freeContext(video_sws_);

  int size = avpicture_layout((AVPicture *)avframe_rgb_, PIX_FMT_RGB24, xsize, ysize, (uint8_t *)RGB, avframe_rgb_size_);
  if (size != avframe_rgb_size_)
  {
    ROS_ERROR("webcam: avpicture_layout error: %d", size);
    return;
  }
}

void UsbCam::process_image(const void * src, int len, camera_image_t *dest)
{
  if (pixelformat_ == V4L2_PIX_FMT_YUYV)
  {
    if (monochrome_)
    { //actually format V4L2_PIX_FMT_Y16, but xioctl gets unhappy if you don't use the advertised type (yuyv)
      mono102mono8((char*)src, dest->image, dest->width * dest->height);
    }
    else
    {
      yuyv2rgb((char*)src, dest->image, dest->width * dest->height);
    }
  }
  else if (pixelformat_ == V4L2_PIX_FMT_UYVY)
    uyvy2rgb((char*)src, dest->image, dest->width * dest->height);
  else if (pixelformat_ == V4L2_PIX_FMT_MJPEG)
    mjpeg2rgb((char*)src, len, dest->image, dest->width * dest->height);
  else if (pixelformat_ == V4L2_PIX_FMT_RGB24)
    rgb242rgb((char*)src, dest->image, dest->width * dest->height);
  else if (pixelformat_ == V4L2_PIX_FMT_GREY)
    memcpy(dest->image, (char*)src, dest->width * dest->height);
}

int UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  if (!buffers_allocated_) {
    error_exit("usb_cam buffer allocation in read_frame");
  }

  switch (io_)
  {
    case IO_METHOD_READ:
      len = read(fd_, buffers_[0].start, buffers_[0].length);
      if (len == -1)
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("read", "read_frame");
        }
      }

      process_image(buffers_[0].start, len, image_);

      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("VIDIOC_DQBUF", "read_frame::IO_METHOD_MMAP");
        }
      }

      assert(buf.index < n_buffers_);
      len = buf.bytesused;
      process_image(buffers_[buf.index].start, len, image_);

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF", "read_frame::IO_METHOD_MMAP");

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("VIDIOC_DQBUF", "read_frame::IO_METHOD_USERPTR");
        }
      }

      for (i = 0; i < n_buffers_; ++i)
        if (buf.m.userptr == (unsigned long)buffers_[i].start && buf.length == buffers_[i].length)
          break;

      assert(i < n_buffers_);
      len = buf.bytesused;
      process_image((void *)buf.m.userptr, len, image_);

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF", "read_frame::IO_METHOD_USERPTR");

      break;
  }

  return 1;
}

bool UsbCam::is_capturing() {
  return is_capturing_;
}

void UsbCam::stop_capturing(void)
{
  if(!is_capturing_) return;

  is_capturing_ = false;
  enum v4l2_buf_type type;

  switch (io_)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF", "stop_capturing");

      break;
  }
}

void UsbCam::start_capturing(void)
{

  if(is_capturing_) return;

  if (!buffers_allocated_) {
    error_exit("usb_cam buffer allocation in start_capturing");
  }

  unsigned int i;
  enum v4l2_buf_type type;

  switch (io_)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF", "start_capturing::IO_METHOD_READ");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON", "start_capturing::IO_METHOD_READ");

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long)buffers_[i].start;
        buf.length = buffers_[i].length;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF", "start_capturing::IO_METHOD_USERPTR");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON", "start_capturing::IO_METHOD_USERPTR");

      break;
  }
  is_capturing_ = true;
}

void UsbCam::uninit_device(void)
{
  unsigned int i;

  if (!buffers_allocated_) {
    error_exit("Attempted to uninit_device without initialized buffers");
  }

  switch (io_)
  {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
        if (-1 == munmap(buffers_[i].start, buffers_[i].length))
          errno_exit("munmap", "uninit_device");
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
        free(buffers_[i].start);
      break;
  }

  free(buffers_);

  buffers_allocated_ = false;
}

void UsbCam::init_read(unsigned int buffer_size)
{
  if (buffers_allocated_) {
    error_exit("Attempted to init_read when buffers were already allocated");
  }

  buffers_ = (buffer*)calloc(1, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  buffers_allocated_ = true;
}

void UsbCam::init_mmap(void)
{
  if (buffers_allocated_) {
    error_exit("Attempted to init_mmap when buffers were already allocated");
  }

  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " does not support memory mapping");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_REQBUFS", "init_mmap");
    }
  }

  if (req.count < 2)
  {
    ROS_ERROR_STREAM("Insufficient buffer memory on " << camera_dev_);
    exit(EXIT_FAILURE);
  }

  buffers_ = (buffer*)calloc(req.count, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_)
  {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (-1 == xioctl(fd_, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF", "init_mmap");

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
				      MAP_SHARED /* recommended */,
				      fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start)
      errno_exit("mmap", "init_mmap");
  }

  buffers_allocated_ = true;
}

void UsbCam::init_userp(unsigned int buffer_size)
{
  if (buffers_allocated_) {
    error_exit("Attempted to init_userp when buffers were already allocated");
  }

  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " does not support "
                "user pointer i/o");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_REQBUFS", "init_userp");
    }
  }

  buffers_ = (buffer*)calloc(4, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_)
  {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start = memalign(/* boundary */page_size, buffer_size);

    if (!buffers_[n_buffers_].start)
    {
      ROS_ERROR("Out of memory");
      exit(EXIT_FAILURE);
    }
  }

  buffers_allocated_ = true;
}

void UsbCam::init_device(int image_width, int image_height, int framerate)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCAP, &cap))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " is no V4L2 device");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_QUERYCAP", "init_device");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    ROS_ERROR_STREAM(camera_dev_ << " is no video capture device");
    exit(EXIT_FAILURE);
  }

  switch (io_)
  {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support read i/o");
        exit(EXIT_FAILURE);
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support streaming i/o");
        exit(EXIT_FAILURE);
      }

      break;
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd_, VIDIOC_CROPCAP, &cropcap))
  {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd_, VIDIOC_S_CROP, &crop))
    {
      switch (errno)
      {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  }
  else
  {
    /* Errors ignored. */
  }

  CLEAR(fmt);

//  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//  fmt.fmt.pix.width = 640;
//  fmt.fmt.pix.height = 480;
//  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
//  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = pixelformat_;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (-1 == xioctl(fd_, VIDIOC_S_FMT, &fmt)) {
    std::string fmtstr(4, ' ');
    uint fmtval = fmt.fmt.pix.pixelformat;
    for (int i = 0; i < 4; i++) {
      fmtstr[i] = (char)(fmtval & 0xFF);
      fmtval >>= 8;
    }
    ROS_ERROR("Error setting VIDIOC_S_FMT to %dx%d %s", fmt.fmt.pix.width, fmt.fmt.pix.height, fmtstr.c_str());
    errno_exit("VIDIOC_S_FMT", "init_device");
  }

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &stream_params) < 0)
    errno_exit("Couldn't query v4l fps!", "init_device");

  ROS_DEBUG("Capability flag: 0x%x", stream_params.parm.capture.capability);

  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = framerate;
  if (xioctl(fd_, VIDIOC_S_PARM, &stream_params) < 0)
    ROS_WARN("Couldn't set camera framerate");
  else
    ROS_DEBUG("Set framerate to be %i", framerate);

  switch (io_)
  {
    case IO_METHOD_READ:
      init_read(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      init_mmap();
      break;

    case IO_METHOD_USERPTR:
      init_userp(fmt.fmt.pix.sizeimage);
      break;
  }
}

void UsbCam::close_device(void)
{
  if (-1 == close(fd_))
    errno_exit("close", "close_device");

  fd_ = -1;
}

void UsbCam::open_device(void)
{
  struct stat st;

  if (-1 == stat(camera_dev_.c_str(), &st))
  {
    ROS_ERROR_STREAM("Cannot identify '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode))
  {
    ROS_ERROR_STREAM(camera_dev_ << " is no device");
    exit(EXIT_FAILURE);
  }

  fd_ = open(camera_dev_.c_str(), O_RDWR /* required */| O_NONBLOCK, 0);

  if (-1 == fd_)
  {
    ROS_ERROR_STREAM("Cannot open '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void UsbCam::start(const std::string& dev, io_method io_method,
		   pixel_format pixel_format, int image_width, int image_height,
		   int framerate)
{
  camera_dev_ = dev;

  io_ = io_method;
  monochrome_ = false;
  if (pixel_format == PIXEL_FORMAT_YUYV)
    pixelformat_ = V4L2_PIX_FMT_YUYV;
  else if (pixel_format == PIXEL_FORMAT_UYVY)
    pixelformat_ = V4L2_PIX_FMT_UYVY;
  else if (pixel_format == PIXEL_FORMAT_MJPEG)
  {
    pixelformat_ = V4L2_PIX_FMT_MJPEG;
    init_mjpeg_decoder(image_width, image_height);
  }
  else if (pixel_format == PIXEL_FORMAT_YUVMONO10)
  {
    //actually format V4L2_PIX_FMT_Y16 (10-bit mono expresed as 16-bit pixels), but we need to use the advertised type (yuyv)
    pixelformat_ = V4L2_PIX_FMT_YUYV;
    monochrome_ = true;
  }
  else if (pixel_format == PIXEL_FORMAT_RGB24)
  {
    pixelformat_ = V4L2_PIX_FMT_RGB24;
  }
  else if (pixel_format == PIXEL_FORMAT_GREY)
  {
    pixelformat_ = V4L2_PIX_FMT_GREY;
    monochrome_ = true;
  }
  else
  {
    ROS_ERROR("Unknown pixel format.");
    exit(EXIT_FAILURE);
  }

  open_device();
  init_device(image_width, image_height, framerate);
  start_capturing();

  image_ = (camera_image_t *)calloc(1, sizeof(camera_image_t));

  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 3;      //corrected 11/10/15 (BYTES not BITS per pixel)

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_->image = (char *)calloc(image_->image_size, sizeof(char));
  memset(image_->image, 0, image_->image_size * sizeof(char));
}

bool UsbCam::start(const std::string& video_device_name, const UsbCamConfig& config) {
  // set the IO method
  UsbCam::io_method io_method = UsbCam::io_method_from_string(config.io_method_name);
  if(io_method == UsbCam::IO_METHOD_UNKNOWN)
  {
    ROS_FATAL("Unknown IO method '%s'", config.io_method_name.c_str());
    return false;
  }

  // set the pixel format
  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(config.pixel_format_name);
  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
  {
    ROS_FATAL("Unknown pixel format '%s'", config.pixel_format_name.c_str());
    return false;
  }

  // start the camera
  start(video_device_name.c_str(), io_method, pixel_format, config.image_width,
       config.image_height, config.framerate);

  applyConfig(config);

  return true;
}

void UsbCam::shutdown(void)
{
  if (is_capturing()) stop_capturing();
  if (buffers_allocated_) uninit_device();
  if (fd_ != -1) close_device();

  if (avcodec_context_)
  {
    avcodec_close(avcodec_context_);
    av_free(avcodec_context_);
    avcodec_context_ = NULL;
  }
  if (avframe_camera_)
    av_free(avframe_camera_);
  avframe_camera_ = NULL;
  if (avframe_rgb_)
    av_free(avframe_rgb_);
  avframe_rgb_ = NULL;
  if(image_)
    free(image_);
  image_ = NULL;
}

void UsbCam::grab_image(sensor_msgs::Image* msg)
{
  // grab the image
  grab_image();
  // stamp the image
  msg->header.stamp = ros::Time::now();
  // fill the info
  if (monochrome_)
  {
    fillImage(*msg, "mono8", image_->height, image_->width, image_->width,
        image_->image);
  }
  else
  {
    fillImage(*msg, "rgb8", image_->height, image_->width, 3 * image_->width,
        image_->image);
  }
}

void UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(fd_ + 1, &fds, NULL, NULL, &tv);

  if (-1 == r)
  {
    if (EINTR == errno)
      return;

    errno_exit("select", "grab_image");
  }

  if (0 == r)
  {
    ROS_ERROR("select timeout");
    exit(EXIT_FAILURE);
  }

  read_frame();
  image_->is_new = 1;
}

// ======================================
// ============== Controls ==============
// ======================================

// enables/disables auto focus
// See https://www.linuxtv.org/downloads/v4l-dvb-apis-old/vidioc-g-ext-ctrls.html
void UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl))
  {
    if (errno != EINVAL)
    {
      perror("VIDIOC_QUERYCTRL");
      return;
    }
    else
    {
      ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
      return;
    }
  }
  else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
    return;
  }
  else
  {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control))
    {
      perror("VIDIOC_S_CTRL");
      return;
    }
  }
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, int value)
{
  set_v4l_parameter(param, boost::lexical_cast<std::string>(value));
}
/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, const std::string& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << camera_dev_ << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  int buffer_size = 256;
  char buffer[buffer_size];
  FILE *stream = popen(cmd.c_str(), "r");
  if (stream)
  {
    while (!feof(stream))
      if (fgets(buffer, buffer_size, stream) != NULL)
        output.append(buffer);
    pclose(stream);
    // any output should be an error
    if (output.length() > 0)
      ROS_WARN("%s", output.c_str());
  }
  else
    ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
}

std::shared_ptr<std::map<std::string, std::shared_ptr<const v4l2_queryctrl>>> UsbCam::getControls(bool refresh) {
  if (controls_.size() == 0 || refresh) {
    // Enumerate all controls
    v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    v4l2_querymenu querymenu;

    queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while (0 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl)) {
      if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        continue;

      std::shared_ptr<v4l2_queryctrl> resultctrl = std::make_shared<v4l2_queryctrl>();
      std::memcpy(resultctrl.get(), &queryctrl, sizeof(queryctrl));
      std::string setting_name = std::string((char*)resultctrl->name);
      controls_.emplace(setting_name, resultctrl);

      if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
        memset(&querymenu, 0, sizeof(querymenu));
        querymenu.id = queryctrl.id;

        auto items = std::make_shared<std::vector<std::string>>();
        for (querymenu.index = queryctrl.minimum;
             querymenu.index <= queryctrl.maximum;
             querymenu.index++) {
          if (0 == xioctl(fd_, VIDIOC_QUERYMENU, &querymenu)) {
            items->push_back(std::string((char*)querymenu.name));
          } else {
            items->push_back(std::string("<invalid>"));
          }
        }
        menu_items_.emplace(setting_name, items);
      }

      queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
  }

  // Populate result map with values from internal representation
  auto result = std::make_shared<std::map<std::string, std::shared_ptr<const v4l2_queryctrl>>>();
  for (auto const& kvp : controls_) {
    result->emplace(kvp.first, kvp.second);
  }
  return result;
}

bool UsbCam::getControlValue(std::shared_ptr<const v4l2_queryctrl> queryctrl, int& value) {
  if (queryctrl->type != V4L2_CTRL_TYPE_INTEGER
      && queryctrl->type != V4L2_CTRL_TYPE_BOOLEAN
      && queryctrl->type != V4L2_CTRL_TYPE_MENU
      && queryctrl->type != V4L2_CTRL_TYPE_INTEGER_MENU) {
    perror("VIDIOC_G_CTRL: Unsupported type");
    return false;
  }

  struct v4l2_control control;

  memset(&control, 0, sizeof(control));
  control.id = queryctrl->id;

  if (-1 == xioctl(fd_, VIDIOC_G_CTRL, &control))
  {
    perror("VIDIOC_G_CTRL");
    return false;
  }

  value = control.value;
  return true;
}

bool UsbCam::setControlValue(std::shared_ptr<const v4l2_queryctrl> queryctrl, int newValue) {
  if (queryctrl->type != V4L2_CTRL_TYPE_INTEGER
      && queryctrl->type != V4L2_CTRL_TYPE_BOOLEAN
      && queryctrl->type != V4L2_CTRL_TYPE_MENU
      && queryctrl->type != V4L2_CTRL_TYPE_INTEGER_MENU) {
    perror("VIDIOC_S_CTRL: Unsupported type");
    return false;
  }

  struct v4l2_control control;

  memset(&control, 0, sizeof(control));
  control.id = queryctrl->id;
  control.value = newValue;

  if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control))
  {
    perror("VIDIOC_S_CTRL");
    return false;
  }

  return true;
}

std::shared_ptr<std::map<std::string, std::shared_ptr<const std::vector<std::string>>>> UsbCam::getMenuItems() {
  if (controls_.size() == 0) {
    getControls();
  }

  // Populate result map with values from internal representation
  auto result = std::make_shared<std::map<std::string, std::shared_ptr<const std::vector<std::string>>>>();
  for (auto const& kvp : menu_items_) {
    result->emplace(kvp.first, kvp.second);
  }
  return result;
}

UsbCam::io_method UsbCam::io_method_from_string(const std::string& str)
{
  if (str == "mmap")
    return IO_METHOD_MMAP;
  else if (str == "read")
    return IO_METHOD_READ;
  else if (str == "userptr")
    return IO_METHOD_USERPTR;
  else
    return IO_METHOD_UNKNOWN;
}

UsbCam::pixel_format UsbCam::pixel_format_from_string(const std::string& str)
{
    if (str == "yuyv")
      return PIXEL_FORMAT_YUYV;
    else if (str == "uyvy")
      return PIXEL_FORMAT_UYVY;
    else if (str == "mjpeg")
      return PIXEL_FORMAT_MJPEG;
    else if (str == "yuvmono10")
      return PIXEL_FORMAT_YUVMONO10;
    else if (str == "rgb24")
      return PIXEL_FORMAT_RGB24;
    else if (str == "grey")
      return PIXEL_FORMAT_GREY;
    else
      return PIXEL_FORMAT_UNKNOWN;
}

void UsbCam::applyConfig(const UsbCamConfig& config) {
  // set camera parameters
  if (config.brightness >= 0)
  {
    set_v4l_parameter("brightness", config.brightness);
  }

  if (config.contrast >= 0)
  {
    set_v4l_parameter("contrast", config.contrast);
  }

  if (config.saturation >= 0)
  {
    set_v4l_parameter("saturation", config.saturation);
  }

  if (config.sharpness >= 0)
  {
    set_v4l_parameter("sharpness", config.sharpness);
  }

  if (config.gain >= 0)
  {
    set_v4l_parameter("gain", config.gain);
  }

  // check auto white balance
  if (config.auto_white_balance_defined) {
    if (config.auto_white_balance)
    {
      set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      set_v4l_parameter("white_balance_temperature_auto", 0);
      set_v4l_parameter("white_balance_temperature", config.white_balance);
    }
  }

  // check auto exposure
  if (config.autoexposure_defined) {
    if (!config.autoexposure)
    {
      // turn down exposure control (from max of 3)
      set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      set_v4l_parameter("exposure_absolute", config.exposure);
    }
  }

  // check auto focus
  if (config.autofocus_defined) {
    if (config.autofocus)
    {
      set_auto_focus(1);
      set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      set_v4l_parameter("focus_auto", 0);
      if (config.focus >= 0)
      {
        set_v4l_parameter("focus_absolute", config.focus);
      }
    }
  }
}


}
