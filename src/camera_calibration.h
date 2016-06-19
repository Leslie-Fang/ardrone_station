#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

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
#define calibration_image_area_width 1280
#define calibration_image_area_height 720

using namespace std;

class Camera_Calibration:public QThread
{
    Q_OBJECT
public:
    Camera_Calibration();
    ~Camera_Calibration();
    void run();

    QImage image;
    QImage image_temp[100];
    bool bool_show_Image;
    bool bool_open_camera;
    unsigned int image_counter;
    unsigned int total_number; //0-99
    int camera_seq;
    int cube_length;
    int cube_width;


public slots:
    bool openCamara();      // 打开摄像头
    void readFarme();       // 读取当前帧信息
    void closeCamara();     // 关闭摄像头。
    void takingPictures();
    void calibration();
    IplImage* QImageToIplImage(QImage * qImage);

private:
    QTimer *timer;
    CvCapture *cam;// 视频获取结构， 用来作为视频获取函数的一个参数
    IplImage* frame_raw;
    IplImage  *frame;//申请IplImage类型指针，就是申请内存空间来存放每一帧图像

    CvMat * intrinsic;
    CvMat * distortion;
    IplImage * mapx;
    IplImage * mapy;

    CvSize board_size;


    void camera_Send_Image()const {emit camera_Image_Signal();}
    void camera_Save_Image()const {emit image_Save_Signal();}

    CvScalar yellow;
    int line_thickness;
    float optical_distance_last;

    bool bool_image_capture;
    bool bool_calibration_done;


signals:
    void camera_Image_Signal()const;
    void image_Save_Signal()const;
};

#endif
