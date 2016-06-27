//*****camera.h********

#ifndef CAMERA_H
#define CAMERA_H

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

#define CV_RED cvScalar(0,0,255,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)
#define CV_BLACK cvScalar(0,0,0,0)
#define CV_YELLOW cvScalar(0,255,255,0)
#define CV_BLUE cvScalar(255,0,0,0)
#define CV_BROWN cvScalar(0,63,127,0)
#define CV_PINK cvScalar(255,170,255,0)

//显示图幅
#define image_area_width 480
#define image_area_height 360

#define raw_image_area_width 720
#define raw_image_area_height 480

using namespace std;
using namespace cv;

class Camera:public QObject
{
    Q_OBJECT
public:
    Camera();
    ~Camera();

    QImage image;
    bool bool_show_Image;
    bool bool_open_camera;
    unsigned int image_counter;

    bool bool_clibration;
    bool bool_clibration_loaded;

    bool bool_fill_color;

    int color_threshold[5][6]; //blue, yellow, red, white, black (h_min, h_max, s_min, s_max, v_min, v_max)

    /*image process*/
    bool camera_left_side;
    bool camera_enemy_side;

    int neg_d;
    int pos_d ;

    int camera_number;


public slots:
    bool openCamara();      // 打开摄像头
    int readFarme();       // 读取当前帧信息
    void closeCamara();     // 关闭摄像头。

private:
    QTimer *timer;
    CvCapture *cam;// 视频获取结构， 用来作为视频获取函数的一个参数
    IplImage  *frame;//申请IplImage类型指针，就是申请内存空间来存放每一帧图像

    CvMat * intrinsic;
    CvMat * distortion;
    IplImage * mapx;
    IplImage * mapy;

    void camera_Send_Image()const {emit camera_Image_Signal();}

    CvScalar yellow;
    int line_thickness;
    float optical_distance_last;

    bool if_read;


    /*position*/
    float robot_image_p[2];
    float robot_real_p[2];

    bool found_field;


    /*functions*/

    void init_paras();

    float point_distance(int x1, int y1, int x2, int y2);

    float point_distance_f(float x1, float y1, float x2, float y2);

    CvMat* find_useful_lines_kb(CvSeq* lines_s, int width, int height,
        int length = 50, float delt_k = 0.2, float delt_d_n = 20, float delt_d_p = 20); // delt_b=(0,1)

    CvPoint2D32f lines_cross_point(float k1, float b1, float k2, float b2);

    void draw_result_lines(CvMat *lines_mat, IplImage* image, CvScalar color = CV_RED, int thickness = 2);

    bool f_euqal(float a, float b);

    CvPoint2D32f point_slant_coordinate_tranlate(float kx, float ky, float x0, float y0);


signals:
    void camera_Image_Signal()const;
};

#endif
