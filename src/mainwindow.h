//******mainwindow.h*******

//bug: 空中飞行时输入航点画实际轨迹有偏离

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string>
#include <QListWidgetItem>
#include <iostream>
#include <fstream>
#include "painterWiget.h"
#include <QTime>
#include <QTimer>
#include <QDateTime>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QListWidgetItem>
#include <QLabel>
#include <QFileDialog>
#include <QPainter>
#include <math.h>
#include <QBitmap>
#include <QPainter>
#include <QMessageBox>
#include <QDir>
#include <QCloseEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>

#define FLY_POSITION_LABEL_WIDTH 360
#define FLY_POSITION_LABEL_HEIGHT 270
#define FLY_ROUTE_LABEL_WIDTH 720
#define FLY_ROUTE_LABEL_HEIGHT 540

#define SUCCESS_COUNTER_INIT 100

#define NORTHERN_HEMISPHERE 1
#define EASTERN_HEMISPHERE 1

#define ROUTE_DRAW_AREA_WIDTH 720
#define ROUTE_DRAW_AREA_HEIGHT 540

#define DEG_TO_RAD 	0.01745329251994
#define RAD_TO_DEG 	57.2957795130823

#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923

#define CONSTANTS_RADIUS_OF_EARTH 6371393

#define MAX_POINT_NUM 1000
#define MAX_DIRACTION_POINT_NUM 200


using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    StatusPainter *status_painter;//用于画图的类指针
    QPixmap *compass_arrow_pixmap;


protected:
    void paintEvent(QPaintEvent *event);
    void closeEvent(QCloseEvent *event);
    void get_Painter_Address(StatusPainter *painter);//用于画图传参的函数

private slots:
    void init_paras();
    void timer_Slot();


    void time_Update();

    void local_Position_Slot();

    float point_dist(float x1, float y1, float x2, float y2);
    float point_line_dist(float m, float n, float k, float b);

    int on_pushButton_Save_Config_clicked();
    int read_saved_paras();


    /*video*/
    void camera_Image_Slot();//摄像头图像显示
    void camera2_Image_Slot();//摄像头图像显示
    void camera_calibration_Image_Slot();
    void calibration_Save_Image_Slot();

    void on_pushButton_Open_Video_clicked();
    void on_pushButton_Close_Video_clicked();

    void on_pushButton_Open_Video_2_clicked();
    void on_pushButton_Close_Video_2_clicked();

    void on_pushButton_Open_Video_Calibration_clicked();
    void on_comboBox_Camera_Calibration_currentIndexChanged(int index);
    void on_pushButton_Close_Video_Calibration_clicked();

    void on_pushButton_Video_Calibration_Save_clicked();

    void on_pushButton_Video_Calibration_Start_clicked();

private:
    Ui::MainWindow *ui;
    QTime system_time;
    QImage image_resize;

    QPoint mouse_pos;


    float preview_scale_width;
    float preview_scale_height;
    int preview_current_num;

    QListWidgetItem *item_cp1;
    QListWidgetItem *item_cp2;
    QListWidgetItem *item_cp3;

    QLabel *fly_position_label;
    QLabel *fly_route_label;


    /*fence*/
    double gps_fence[MAX_POINT_NUM][3]; //(lat, lon, initial_sequence)
    double gps_fence_cp1[MAX_POINT_NUM][3];
    double gps_fence_cp2[MAX_POINT_NUM][3];
    double gps_fence_cp3[MAX_POINT_NUM][3];

    int gps_num;//start from 0
    int gps_num_cp1;
    int gps_num_cp2;
    int gps_num_cp3;   
    float gps_fence_local[MAX_POINT_NUM][2]; //local: East->x, North->y

    /*diraction*/
    double gps_diraction[MAX_DIRACTION_POINT_NUM][2]; //(lat, lon)
    float diraction_k;
    int diraction_p_num;

    /*home position*/
    double home_lat;
    double home_lon;

    /*distance between lines*/
    float dist_between_lines;

    /*intersection points, local*/
    float intersection_p_local[MAX_POINT_NUM][2];
    float route_p_local[MAX_POINT_NUM][2];//local: East->x, North->y
    double route_p_gps[MAX_POINT_NUM][2];

    int intersection_num;

    /*for common flight mode*/
    float common_length;
    float common_width;
    float common_height;
    int common_times;
    bool common_side; //left: true; right: false
    bool common_mode; //used when drawing route to calculate different painting scale

    /*for break point*/
    bool break_point_flag1;
    bool break_point_flag2;
    int intersection_num_last;
    double route_p_gps_last[MAX_POINT_NUM][2];
    int gps_num_last;
    double gps_fence_last[MAX_POINT_NUM][3];
    double break_point_lat;
    double break_point_lon;
    int break_position_num;
    double route_p_gps_read[MAX_POINT_NUM][2];
    float route_p_local_read[MAX_POINT_NUM][2];
    int route_p_num_read;
    double break_point_lat_read;
    double break_point_lon_read;
    int break_point_seq_read;


    /*route offset*/
    float offset_angle_d;
    float offset_dist_m;
    float measure_compensation_m;
    float spray_length;
    float spray_width;

    /*item sequence for listwidget*/
    int list_seq;
    int list_seq_cp1;
    int list_seq_cp2;
    int list_seq_cp3;

    /*others*/
    int success_counter;

    int controller_flag;
    int controller_flag_last;

    double orientation_last;

    bool bool_flying;

    bool controller_working;

    float flying_height;
    float take_off_height;

    int flying_time;
    unsigned int flying_status_counter;
    unsigned int flying_status_counter_last;

    int time_counter;

    bool battery_low;

    int route_plan_mode; // 0: GPS fence mode 1:common mode 2:GPS fence break point mode 3:common break point mode

    //以下变量用于画路径图
    float paint_scale ;
    float real_position[6000][2];
    int position_num;
    int save_counter;

    float fly_distance;

};

#endif // MAINWINDOW_H
