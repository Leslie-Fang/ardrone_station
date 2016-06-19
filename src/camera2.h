//*****camera.h********

#ifndef CAMERA2_H
#define CAMERA2_H

#include <QThread>
#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QTime>
#include <QDateTime>
#include <QDir>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <string.h>


//显示图幅
#define image_area_width 480
#define image_area_height 360


using namespace std;

class Camera2:public QThread
{
    Q_OBJECT
public:
    Camera2();
    ~Camera2();
    void run();

    QImage image;
    bool bool_show_Image;
    bool bool_open_camera;
    bool bool_show_ruler;
    unsigned int image_counter;

public slots:
    void openCamara();      // 打开摄像头
    void readFarme();       // 读取当前帧信息
    void closeCamara();     // 关闭摄像头。


private:
    QTimer *timer;
    CvCapture *cam;// 视频获取结构， 用来作为视频获取函数的一个参数
    IplImage  *frame;//申请IplImage类型指针，就是申请内存空间来存放每一帧图像


    void camera_Send_Image()const {emit camera_Image_Signal();}

    CvScalar yellow;
    int line_thickness;
    float optical_distance_last;


signals:
    void camera_Image_Signal()const;
};

#endif
