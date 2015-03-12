#include "v4l.hpp"

namespace  {
int format_properties(const unsigned int format,
        const unsigned int width,
        const unsigned int height,
        size_t*linewidth,
        size_t*framewidth) {
    size_t lw, fw;
    switch(format) {
    case V4L2_PIX_FMT_YUV420: case V4L2_PIX_FMT_YVU420:
        lw = width; /* ??? */
        fw = ROUND_UP_4 (width) * ROUND_UP_2 (height);
        fw += 2 * ((ROUND_UP_8 (width) / 2) * (ROUND_UP_2 (height) / 2));
    break;
    case V4L2_PIX_FMT_UYVY: case V4L2_PIX_FMT_Y41P: case V4L2_PIX_FMT_YUYV: case V4L2_PIX_FMT_YVYU:
        lw = (ROUND_UP_2 (width) * 2);
        fw = lw * height;
    break;
    default:
        return 0;
    }

    if(linewidth)*linewidth=lw;
    if(framewidth)*framewidth=fw;

    return 1;
}

void print_format(struct v4l2_format*vid_format) {
    printf("	vid_format->type                =%d\n",	vid_format->type );
    printf("	vid_format->fmt.pix.width       =%d\n",	vid_format->fmt.pix.width );
    printf("	vid_format->fmt.pix.height      =%d\n",	vid_format->fmt.pix.height );
    printf("	vid_format->fmt.pix.pixelformat =%d\n",	vid_format->fmt.pix.pixelformat);
    printf("	vid_format->fmt.pix.sizeimage   =%d\n",	vid_format->fmt.pix.sizeimage );
    printf("	vid_format->fmt.pix.field       =%d\n",	vid_format->fmt.pix.field );
    printf("	vid_format->fmt.pix.bytesperline=%d\n",	vid_format->fmt.pix.bytesperline );
    printf("	vid_format->fmt.pix.colorspace  =%d\n",	vid_format->fmt.pix.colorspace );
}

}

/**********************************************************************/

V4lInterface::~V4lInterface(){
    close(fdwr);
    free(buffer);
}

V4lInterface::V4lInterface(std::string nameDevice, cv::Size frameSize) {
    m_nameDevice = nameDevice;
    m_frameSize = frameSize;
    fdwr = 0;
    ret_code = 0;

    fdwr = open(m_nameDevice.c_str(), O_RDWR);
    assert(fdwr >= 0);
    ret_code = ioctl(fdwr, VIDIOC_QUERYCAP, &vid_caps);
    assert(ret_code != -1);

    memset(&vid_format, 0, sizeof(vid_format));

    ret_code = ioctl(fdwr, VIDIOC_G_FMT, &vid_format);
    print_format(&vid_format);

    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    vid_format.fmt.pix.width = m_frameSize.width;
    vid_format.fmt.pix.height = m_frameSize.height;
    vid_format.fmt.pix.pixelformat = FRAME_FORMAT;
    vid_format.fmt.pix.sizeimage = framesize;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;
    vid_format.fmt.pix.bytesperline = linewidth;
    vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    ret_code = ioctl(fdwr, VIDIOC_S_FMT, &vid_format);
    assert(ret_code != -1);

    printf("frame: format=%d\tsize=%d\n", FRAME_FORMAT, framesize);
    print_format(&vid_format);

    if(!format_properties(vid_format.fmt.pix.pixelformat,
                      vid_format.fmt.pix.width, vid_format.fmt.pix.height,
                      &linewidth,
                      &framesize)) {
        printf("unable to guess correct settings for format '%d'\n", FRAME_FORMAT);
    }

    buffer=(__u8*)malloc(sizeof(__u8)*framesize);
}

void V4lInterface::putImage(const cv::Mat &im1){
    memset(buffer, 0, framesize);

    cv::Mat yuv;
    cv::cvtColor(im1, yuv, cv::COLOR_BGR2YUV_YV12 );

    for (int i = 0; i < framesize; ++i) {
        buffer[i] = im1.data[i];
    }

    write(fdwr, buffer, framesize);
}

/**********************************************************************/
