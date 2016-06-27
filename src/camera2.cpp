//***********camera2.cpp***********

#include "camera2.h"
#include "stdio.h"
#include <string>
#include <iostream>
#include <math.h>

using namespace std;


Camera2::Camera2()
{
    cam = NULL;
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(readFarme()));

    bool_show_Image=true;

    line_thickness=2;
    yellow=cvScalar(0,255,255);
}

Camera2::~Camera2()
{
    delete timer;
}


bool Camera2::openCamara()
{
    cam = cvCreateCameraCapture(0);//打开摄像头，从摄像头中获取视频

    if(cam == 0)
    {
        bool_open_camera=false;
        return false;
    }
    else
    {
        //设定捕获图像大小及帧率
        cvSetCaptureProperty(cam,CV_CAP_PROP_FPS,30);
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_WIDTH,raw_image_area_width_2);
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_HEIGHT,raw_image_area_height_2);

        timer->start(33);              // 开始计时，超时则发出timeout()信号，30帧/s
        bool_open_camera=true;

        return true;
    }
}

void Camera2::readFarme()
{
    //cvNamedWindow("video",1);
    IplImage* frame_raw = cvQueryFrame(cam);// 从摄像头中抓取并返回每一帧

    //窗口大小适应
    frame=cvCreateImage(cvSize(image_area_width,image_area_height),8,3);//4:3画面
    cvResize(frame_raw,frame,CV_INTER_NN);

     // 将抓取到的帧，转换为QImage格式。QImage::Format_RGB888不同的摄像头用不同的格式。
    image = QImage((const uchar*)frame->imageData, frame->width, frame->height,QImage::Format_RGB888).rgbSwapped();

    if(bool_show_Image)camera_Send_Image();


    cvReleaseImage(&frame);//释放图像占用的内存
}

void Camera2::closeCamara()
{
    timer->stop();         // 停止读取数据。
    cvReleaseCapture(&cam);//释放内存；
    bool_open_camera=false;

    //测试代码
    //cvDestroyWindow("TEST");
}

