#ifndef _V4L_HPP
#define _V4L_HPP

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>

#include <iostream>
#include <ctype.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define FRAME_FORMAT V4L2_PIX_FMT_YVU420

#define ROUND_UP_2(num)  (((num)+1)&~1)
#define ROUND_UP_4(num)  (((num)+3)&~3)
#define ROUND_UP_8(num)  (((num)+7)&~7)
#define ROUND_UP_16(num) (((num)+15)&~15)
#define ROUND_UP_32(num) (((num)+31)&~31)
#define ROUND_UP_64(num) (((num)+63)&~63)


class V4lInterface
{
public:
    V4lInterface( std::string nameDevice, cv::Size frameSize = cv::Size());

    ~V4lInterface();

    void putImage( const cv::Mat& im1 );

private:
    std::string m_nameDevice;
    cv::Size m_frameSize;
    struct v4l2_capability vid_caps;
    struct v4l2_format vid_format;
    size_t framesize;
    size_t linewidth;
    __u8*buffer;
    int fdwr;
    int ret_code;

};

#endif
