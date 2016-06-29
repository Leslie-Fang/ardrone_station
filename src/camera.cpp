//***********camera.cpp***********

#include "camera.h"
#include "stdio.h"
#include <string>
#include <iostream>

using namespace std;

float cross_points_real_position_right_enemy[5][5][2] = {
    5.f, 2.f, 5.f, 1.f, 5.f, 0.f, 5.f, -1.f, 5.f, -2.f,
    4.f, 2.f, 4.f, 1.f, 4.f, 0.f, 4.f, -1.f, 4.f, -2.f,
    3.f, 2.f, 3.f, 1.f, 3.f, 0.f, 3.f, -1.f, 3.f, -2.f,
    2.f, 2.f, 2.f, 1.f, 2.f, 0.f, 2.f, -1.f, 2.f, -2.f,
    1.f, 2.f, 1.f, 1.f, 1.f, 0.f, 1.f, -1.f, 1.f, -2.f
};

float cross_points_real_position_left_enemy[5][5][2];

Camera::Camera()
{
    //camera_number = 0;

    cam = NULL;
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(readFarme()));

    bool_show_Image = true;
    bool_fill_color = false;
    bool_clibration_loaded = false;

    line_thickness=2;
    yellow=cvScalar(0,255,255);

    bool load_1, load_2;
    QFile file("/home/chg/catkin_ws/src/ardrone_station/parameters/1_Intrinsic.xml");
    if (file.exists()){
        intrinsic=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/1_Intrinsic.xml");
        load_1 = true;
    }
    else load_1 = false;

    QFile file2("/home/chg/catkin_ws/src/ardrone_station/parameters/1_Distortion.xml");
    if (file2.exists()){
        distortion=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/1_Distortion.xml");
        load_2 = true;
    }
    else load_2 = false;

    if(load_1 && load_2)
    {
        bool_clibration = true;
    }
    else bool_clibration = false;

    intrinsic=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/1_Intrinsic.xml");
    distortion=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/1_Distortion.xml");


    init_paras();

}

Camera::~Camera()
{
    delete timer;
}

void Camera::init_paras()
{

    capture = false;

    //camera_left_side = false;
    camera_enemy_side = true;
    position_clibration_done = false;

    height_threshold = 0;

    neg_d = 20;
    pos_d = 20;

    found_field = false;

    robot_image_p[0] = 0.f;
    robot_image_p[1] = 0.f;
    robot_real_p[0] = 0.f;
    robot_real_p[1] = 0.f;


    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            cross_points_real_position_left_enemy[i][j][0] = cross_points_real_position_right_enemy[i][4-j][0];
            cross_points_real_position_left_enemy[i][j][1] = cross_points_real_position_right_enemy[i][4-j][1];
        }
    }
}

bool Camera::openCamara()
{
    cam = cvCreateCameraCapture(camera_number);//打开摄像头，从摄像头中获取视频

    if(cam == 0)
    {
        bool_open_camera=false;
        return false;
    }
    else
    {
        //设定捕获图像大小及帧率
        cvSetCaptureProperty(cam,CV_CAP_PROP_FPS,30);
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_WIDTH,raw_image_area_width);
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_HEIGHT,raw_image_area_height);

        timer->start(33);              // 开始计时，超时则发出timeout()信号，30帧/s
        bool_open_camera=true;

        return true;
    }

}


int Camera::readFarme()
{
    //cvNamedWindow("video",1);
    IplImage* frame_raw = cvQueryFrame(cam);// 从摄像头中抓取并返回每一帧

    if(bool_clibration)
    {
        if(!bool_clibration_loaded)
        {
            mapx=cvCreateImage(cvGetSize(frame_raw),IPL_DEPTH_32F,1);
            mapy=cvCreateImage(cvGetSize(frame_raw),IPL_DEPTH_32F,1);
            cvInitUndistortMap(intrinsic,distortion,mapx,mapy);
            bool_clibration_loaded = true;

            cout<<"clibration parameters loaded!"<<endl;
        }

        cvRemap(frame_raw,frame_raw,mapx,mapy);
    }

    CvScalar s_raw;

    for(int i = 0;i < frame_raw->height;i++)
    {
        for(int j = 0;j < frame_raw->width;j++)
        {
            s_raw = cvGet2D(frame_raw,i,j); // get the (i,j) pixel value
            if(i < (float)height_threshold/100.f*raw_image_area_height)
            {
                s_raw.val[0] = 169;
                s_raw.val[1] = 169;
                s_raw.val[2] = 169;
                cvSet2D(frame_raw,i,j,s_raw);//set the (i,j) pixel value
            }

        }
    }


    //创建灰色图像
    IplImage* fill_color = cvCreateImage(cvGetSize(frame_raw), 8, 3);
    CvScalar s;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            s.val[0]=169;
            s.val[1]=169;
            s.val[2]=169;
            cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
        }
    }

    //hsv空间下分离
    IplImage* hsv = cvCreateImage(cvGetSize(frame_raw), 8, 3);
    cvCvtColor(frame_raw, hsv, CV_BGR2HSV);

    if(bool_fill_color)
    {
        //Blue
        CvScalar s_blue;

        for(int i = 0;i < fill_color->height;i++)
        {
            for(int j = 0;j < fill_color->width;j++)
            {
                s_blue = cvGet2D(hsv,i,j); // get the (i,j) pixel value
                if(s_blue.val[0] >= color_threshold[0][0] && s_blue.val[0] <= color_threshold[0][1] &&
                        s_blue.val[1] >= color_threshold[0][2] && s_blue.val[1] <= color_threshold[0][3] &&
                        s_blue.val[2] >= color_threshold[0][4] && s_blue.val[2] <= color_threshold[0][5])
                {
                    s.val[0] = 255;
                    s.val[1] = 144;
                    s.val[2] = 30;
                    cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
                }

            }
        }

        //Yellow
        CvScalar s_yellow;

        for(int i = 0;i < fill_color->height;i++)
        {
            for(int j = 0;j < fill_color->width;j++)
            {
                s_yellow = cvGet2D(hsv,i,j); // get the (i,j) pixel value
                if(s_yellow.val[0] >= color_threshold[1][0] && s_yellow.val[0] <= color_threshold[1][1] &&
                        s_yellow.val[1] >= color_threshold[1][2] && s_yellow.val[1] <= color_threshold[1][3] &&
                        s_yellow.val[2] >= color_threshold[1][4] && s_yellow.val[2] <= color_threshold[1][5])
                {
                    s.val[0] = 86;
                    s.val[1] = 255;
                    s.val[2] = 255;
                    cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
                }

            }
        }
    }

    //Red
    CvScalar s_red;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_red = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_red.val[0] >= color_threshold[2][0] && s_red.val[0] <= color_threshold[2][1] &&
                    s_red.val[1] >= color_threshold[2][2] && s_red.val[1] <= color_threshold[2][3] &&
                    s_red.val[2] >= color_threshold[2][4] && s_red.val[2] <= color_threshold[2][5])
            {
                s.val[0] = 0;
                s.val[1] = 0;
                s.val[2] = 255;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }

    //White
    /*CvScalar s_white;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_white = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_white.val[0] >= color_threshold[3][0] && s_white.val[0] <= color_threshold[3][1] &&
                    s_white.val[1] >= color_threshold[3][2] && s_white.val[1] <= color_threshold[3][3] &&
                    s_white.val[2] >= color_threshold[3][4] && s_white.val[2] <= color_threshold[3][5])
            {
                s.val[0] = 255;
                s.val[1] = 255;
                s.val[2] = 255;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }

    //Black
    CvScalar s_black;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_black = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_black.val[0] >= color_threshold[4][0] && s_black.val[0] <= color_threshold[4][1] &&
                    s_black.val[1] >= color_threshold[4][2] && s_black.val[1] <= color_threshold[4][3] &&
                    s_black.val[2] >= color_threshold[4][4] && s_black.val[2] <= color_threshold[4][5])
            {
                s.val[0] = 0;
                s.val[1] = 0;
                s.val[2] = 0;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }*/

    /***Find robot***/
    if(position_clibration_done)
    {

        CvScalar s_red_black;
        IplImage* red = cvCreateImage(cvGetSize(fill_color), 8, 1);

        for(int i = 0;i < red->height;i++)
        {
            for(int j = 0;j < red->width;j++)
            {
                s_red_black = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
                if(s_red_black.val[0] == 0 && s_red_black.val[1] == 0 && s_red_black.val[2] == 255)
                {
                    cvSet2D(red,i,j,CV_WHITE);//set the (i,j) pixel value
                }
                else
                {
                    cvSet2D(red,i,j,CV_BLACK);//set the (i,j) pixel value
                }
            }
        }


        cvErode( red, red, NULL, 1);
        cvDilate( red, red, NULL, 1);

        CvPoint2D32f robot_center;
        float radius;

        CvMemStorage* red_storage = cvCreateMemStorage(0);
        CvSeq* red_contours = 0;
        int contour_num = cvFindContours(red, red_storage, &red_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cout<<"contour_num="<<contour_num<<endl;
        if(contour_num > 0)
        {
            cvDrawContours(fill_color, red_contours, CV_GREEN, CV_GREEN, 50);

            CvSeq *c = 0;
            bool found = false;
            int counter = 0;
            robot_image_p[0] = 0.f;
            robot_image_p[1] = 0.f;
            for (c = red_contours;c !=NULL;c = c->h_next)
            {
                double area = fabs(cvContourArea(c,CV_WHOLE_SEQ));
                if(area > 200)
                {
                    cvMinEnclosingCircle(c,&robot_center,&radius);
                    cvCircle(fill_color,cvPointFrom32f(robot_center),4,CV_GREEN,4);
                    robot_image_p[0] += robot_center.x;
                    robot_image_p[1] += raw_image_area_height - robot_center.y;
                    found = true;
                    counter ++;
                }

            }
            if(!found)
            {
                robot_image_p[0] = 0.f;
                robot_image_p[1] = 0.f;
            }
            else
            {
                robot_image_p[0] = robot_image_p[0] / counter;
                robot_image_p[1] = robot_image_p[1] / counter;
            }
        }
        else
        {
            robot_image_p[0] = 0.f;
            robot_image_p[1] = 0.f;
        }

        cvReleaseMemStorage(&red_storage);
        cvReleaseImage(&red);


        /***Calculate real position ***/
        if(robot_image_p[0] > 0.001) //image_position
        {
            //rank crosspoints by distance to robot, from small to large
            float distance_temp[5][5];
            for(int i = 0; i < 5; i++)
            {
                for(int j = 0; j < 5; j++)
                {
                    distance_temp[i][j] = point_distance_f(robot_image_p[0], robot_image_p[1], cross_points_position_enemy[i][j][0], cross_points_position_enemy[i][j][1]);
                }
            }
            //nearest point
            float min_distance = 10000.f;
            int min_row, min_col;
            for(int i = 0; i < 5; i++)
            {
                for(int j = 0; j < 5; j++)
                {
                    if(distance_temp[i][j] < min_distance)
                    {
                        min_distance = distance_temp[i][j];
                        min_row = i;
                        min_col = j;
                    }
                }
            }
            //second nearest point with different row and col
            float min_distance_2 = 10000.f;
            int min_row_2, min_col_2;
            for(int i = 0; i < 5; i++)
            {
                for(int j = 0; j < 5; j++)
                {
                    if(distance_temp[i][j] < min_distance_2 && distance_temp[i][j] > min_distance && i!= min_row && j!=min_col)
                    {
                        min_distance_2 = distance_temp[i][j];
                        min_row_2 = i;
                        min_col_2 = j;
                    }
                }
            }


            if(!camera_left_side)
            {
                //when right side
                CvPoint2D32f min_dist_p1_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, cross_points_position_enemy[min_row][min_col][0], cross_points_position_enemy[min_row][min_col][1]);
                CvPoint2D32f min_dist_p2_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, cross_points_position_enemy[min_row_2][min_col_2][0], cross_points_position_enemy[min_row_2][min_col_2][1]);
                CvPoint2D32f robot_image_p_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, robot_image_p[0], robot_image_p[1]);

                float kx = (cross_points_real_position_right_enemy[min_row][min_col][0] - cross_points_real_position_right_enemy[min_row_2][min_col_2][0]) / (min_dist_p1_tr.x - min_dist_p2_tr.x);
                float ky = (cross_points_real_position_right_enemy[min_row][min_col][1] - cross_points_real_position_right_enemy[min_row_2][min_col_2][1]) / (min_dist_p1_tr.y - min_dist_p2_tr.y);

                robot_real_p[0] = (cross_points_real_position_right_enemy[min_row][min_col][0] + kx*(robot_image_p_tr.x - min_dist_p1_tr.x) + cross_points_real_position_right_enemy[min_row_2][min_col_2][0] + kx*(robot_image_p_tr.x - min_dist_p2_tr.x))/2.f;
                robot_real_p[1] = (cross_points_real_position_right_enemy[min_row][min_col][1] + ky*(robot_image_p_tr.y - min_dist_p1_tr.y) + cross_points_real_position_right_enemy[min_row_2][min_col_2][1] + ky*(robot_image_p_tr.y - min_dist_p2_tr.y))/2.f;
            }
            else
            {
                //when left side
                cout<<"negative_k_average "<<negative_k_average<<"positive_k_average"<<positive_k_average<<"cross_points_position_enemy"<<cross_points_position_enemy[min_row][min_col][0]<<" , "<<cross_points_position_enemy[min_row][min_col][1]<<endl;
                CvPoint2D32f min_dist_p1_tr = point_slant_coordinate_tranlate(negative_k_average, positive_k_average, cross_points_position_enemy[min_row][min_col][0], cross_points_position_enemy[min_row][min_col][1]);
                CvPoint2D32f min_dist_p2_tr = point_slant_coordinate_tranlate(negative_k_average, positive_k_average, cross_points_position_enemy[min_row_2][min_col_2][0], cross_points_position_enemy[min_row_2][min_col_2][1]);
                CvPoint2D32f robot_image_p_tr = point_slant_coordinate_tranlate(negative_k_average, positive_k_average, robot_image_p[0], robot_image_p[1]);

                cout<<"px = "<<min_dist_p1_tr.x<<" , py = "<<min_dist_p1_tr.y<<endl;
                cout<<"rx = "<<robot_image_p_tr.x<<" , ry = "<<robot_image_p_tr.y<<endl;

                float kx = (cross_points_real_position_left_enemy[min_row][min_col][0] - cross_points_real_position_left_enemy[min_row_2][min_col_2][0]) / (min_dist_p1_tr.x - min_dist_p2_tr.x);
                float ky = (cross_points_real_position_left_enemy[min_row][min_col][1] - cross_points_real_position_left_enemy[min_row_2][min_col_2][1]) / (min_dist_p1_tr.y - min_dist_p2_tr.y);
                cout<<"kx = "<<kx<<" , ky = "<<ky<<endl;

                robot_real_p[0] = (cross_points_real_position_left_enemy[min_row][min_col][0] + kx*(robot_image_p_tr.x - min_dist_p1_tr.x) + cross_points_real_position_left_enemy[min_row_2][min_col_2][0] + kx*(robot_image_p_tr.x - min_dist_p2_tr.x))/2.f;
                robot_real_p[1] = (cross_points_real_position_left_enemy[min_row][min_col][1] + ky*(robot_image_p_tr.y - min_dist_p1_tr.y) + cross_points_real_position_left_enemy[min_row_2][min_col_2][1] + ky*(robot_image_p_tr.y - min_dist_p2_tr.y))/2.f;
            }

            //position offset


            //cout<<"Point1 Position = ("<<cross_points_real_position_right_enemy[min_row][min_col][0]<<","<<cross_points_real_position_right_enemy[min_row][min_col][1]<<")\n";
            //cout<<"Point2 Position = ("<<cross_points_real_position_right_enemy[min_row_2][min_col_2][0]<<","<<cross_points_real_position_right_enemy[min_row_2][min_col_2][1]<<")\n";
            cout<<"Robot image Position = ("<<robot_image_p[0]<<","<<robot_image_p[1]<<")\n";
            cout<<"Robot Real Position = ("<<robot_real_p[0]<<","<<robot_real_p[1]<<")\n";
            cvCircle(fill_color, cvPoint(cross_points_position_enemy[min_row][min_col][0], raw_image_area_height - cross_points_position_enemy[min_row][min_col][1]), 4, CV_GREEN, 6);
            cvCircle(fill_color, cvPoint(cross_points_position_enemy[min_row_2][min_col_2][0], raw_image_area_height - cross_points_position_enemy[min_row_2][min_col_2][1]), 4, CV_GREEN, 6);
            cvCircle(fill_color, cvPoint((int)robot_image_p[0], raw_image_area_height - (int)robot_image_p[1]), 10, CV_RED, 2);
        }
        else
        {
            cout<<"Can not find robot!!"<<endl;
            robot_real_p[0] = -1000.f;
            robot_real_p[1] = -1000.f;
        }

    }

    /***Lines Process***/
    if(capture)
    {
        cvSaveImage("/home/chg/catkin_ws/src/1.jpg",frame_raw);
        capture = false;
        auto_position();
    }

    /***Display**/
    if(!bool_fill_color)
    {
         // 将抓取到的帧，转换为QImage格式。QImage::Format_RGB888不同的摄像头用不同的格式。
        image = QImage((const uchar*)frame_raw->imageData, frame_raw->width, frame_raw->height,QImage::Format_RGB888).rgbSwapped();

        if(bool_show_Image)camera_Send_Image();
    }
    else
    {


         // 将抓取到的帧，转换为QImage格式。QImage::Format_RGB888不同的摄像头用不同的格式。
        image = QImage((const uchar*)fill_color->imageData, fill_color->width, fill_color->height,QImage::Format_RGB888).rgbSwapped();

        if(bool_show_Image)camera_Send_Image();
    }

    cvReleaseImage(&fill_color);
    cvReleaseImage(&hsv);

    return 0;
}

int Camera::auto_position()
{
    char image_name[100] = "/home/chg/catkin_ws/src/1.jpg";
    IplImage* frame_raw = cvLoadImage(image_name);
    cout<<"Loaded!\n";


    //创建灰色图像
    IplImage* fill_color = cvCreateImage(cvGetSize(frame_raw), 8, 3);
    CvScalar s;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            s.val[0]=169;
            s.val[1]=169;
            s.val[2]=169;
            cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
        }
    }

    //hsv空间下分离
    IplImage* hsv = cvCreateImage(cvGetSize(frame_raw), 8, 3);
    cvCvtColor(frame_raw, hsv, CV_BGR2HSV);

    //Blue
    CvScalar s_blue;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_blue = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_blue.val[0] >= color_threshold[0][0] && s_blue.val[0] <= color_threshold[0][1] &&
                    s_blue.val[1] >= color_threshold[0][2] && s_blue.val[1] <= color_threshold[0][3] &&
                    s_blue.val[2] >= color_threshold[0][4] && s_blue.val[2] <= color_threshold[0][5])
            {
                s.val[0] = 255;
                s.val[1] = 144;
                s.val[2] = 30;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }

    //Yellow
    CvScalar s_yellow;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_yellow = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_yellow.val[0] >= color_threshold[1][0] && s_yellow.val[0] <= color_threshold[1][1] &&
                    s_yellow.val[1] >= color_threshold[1][2] && s_yellow.val[1] <= color_threshold[1][3] &&
                    s_yellow.val[2] >= color_threshold[1][4] && s_yellow.val[2] <= color_threshold[1][5])
            {
                s.val[0] = 86;
                s.val[1] = 255;
                s.val[2] = 255;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }


    //Red
    CvScalar s_red;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_red = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_red.val[0] >= color_threshold[2][0] && s_red.val[0] <= color_threshold[2][1] &&
                    s_red.val[1] >= color_threshold[2][2] && s_red.val[1] <= color_threshold[2][3] &&
                    s_red.val[2] >= color_threshold[2][4] && s_red.val[2] <= color_threshold[2][5])
            {
                s.val[0] = 0;
                s.val[1] = 0;
                s.val[2] = 255;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }



    cout<<"Find yellow lines\n";
    /*****Find yellow lines ******/
    CvScalar s_yellow_black;
    IplImage* yellow = cvCreateImage(cvGetSize(fill_color), 8, 1);

    for(int i = 0;i < yellow->height;i++)
    {
        for(int j = 0;j < yellow->width;j++)
        {
            s_yellow_black = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            if(s_yellow_black.val[0] == 86 && s_yellow_black.val[1] == 255 && s_yellow_black.val[2] == 255)
            {
                cvSet2D(yellow,i,j,CV_WHITE);//set the (i,j) pixel value
            }
            else
            {
                cvSet2D(yellow,i,j,CV_BLACK);//set the (i,j) pixel value
            }
        }
    }

    cvErode( yellow,yellow, NULL, 1);
    cvDilate( yellow,yellow, NULL, 1);

    //find lines
    IplImage* yellow_canny = cvCreateImage(cvGetSize(yellow), 8, 1);
    cvCanny(yellow, yellow_canny, 100, 200, 3);

    CvMemStorage* storage_yellow_canny = cvCreateMemStorage(0);
    CvSeq* lines_yellow = 0;

    IplImage* color_yellow = cvCreateImage(cvGetSize(yellow_canny), 8, 3);
    cvCvtColor( yellow_canny, color_yellow, CV_GRAY2BGR );

    lines_yellow = cvHoughLines2(yellow_canny, storage_yellow_canny, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 60, 50, 40);
    cout<<"total lines "<<lines_yellow->total<<endl;

    /**Draw lines**/
   /* for (int i = 0; i < lines_yellow->total; i++)
    {
        //point: line[0] and line[1]
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines_yellow, i);
        cvLine(color_yellow, line[0], line[1], CV_RED, 1, CV_AA, 0);
    }


    /***Find the useful lines***/
    CvMat* yellow_lines_mat = find_useful_lines_kb(lines_yellow, raw_image_area_width, raw_image_area_height, 50, 0.1, neg_d, pos_d);
    draw_result_lines(yellow_lines_mat, fill_color, CV_YELLOW);


    /***Classify negtive k lines and positive k lines***/
    int yellow_lines_total = (int)CV_MAT_ELEM(*yellow_lines_mat, float, 0, 0 );

    float negative_yellow_lines_array[yellow_lines_total][2];  //k,b
    float positive_yellow_lines_array[yellow_lines_total][2];  //k,b
    int negative_k_total = 0;
    int positive_k_total = 0;

    negative_k_average = 0.f;
    positive_k_average = 0.f;

    for(int i = 1; i <= yellow_lines_total; i++)
    {
        if(CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 ) < 0.f)
        {
            negative_yellow_lines_array[negative_k_total][0] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 );
            negative_yellow_lines_array[negative_k_total][1] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 1 );
            negative_k_average += negative_yellow_lines_array[negative_k_total][0];
            negative_k_total ++;
            //cout<<"nk ("<<negative_yellow_lines_array[negative_k_total][0]<<","<<negative_yellow_lines_array[negative_k_total][1]<<")\n";
        }
    }
    if(negative_k_total > 1) negative_k_average = negative_k_average / negative_k_total;

    for(int i = 1; i <= yellow_lines_total; i++)
    {
        if(CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 ) >= 0.f)
        {
            positive_yellow_lines_array[positive_k_total][0] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 );
            positive_yellow_lines_array[positive_k_total][1] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 1 );
            positive_k_average += positive_yellow_lines_array[positive_k_total][0];
            positive_k_total ++;
            //cout<<"pk ("<<positive_yellow_lines_array[i-1][0]<<","<<positive_yellow_lines_array[i-1][1]<<")\n";
        }
    }
    if(positive_k_total > 1) positive_k_average = positive_k_average / positive_k_total;

    cout<<"negative k lines: "<<negative_k_total<<endl;
    cout<<"positive k lines: "<<positive_k_total<<endl;

    if(negative_k_total < 3 || positive_k_total < 3) {
        cout<<"Can not find competetion field!"<<endl;
        found_field = false;
    }
    else found_field = true;

    //When there is right lines
    if(found_field)
    {
        /***Rank array by |k|, from small to large***/
        for(int i = 0; i < negative_k_total - 1; i++)
        {
            for(int j = i + 1; j < negative_k_total; j++)
            {
                if(fabs(negative_yellow_lines_array[i][0]) > fabs(negative_yellow_lines_array[j][0]))
                {
                    float temp1 = negative_yellow_lines_array[j][1];
                    float temp2 = negative_yellow_lines_array[j][0];
                    negative_yellow_lines_array[j][1] = negative_yellow_lines_array[i][1];
                    negative_yellow_lines_array[j][0] = negative_yellow_lines_array[i][0];
                    negative_yellow_lines_array[i][1] = temp1;
                    negative_yellow_lines_array[i][0] = temp2;
                }
            }
        }

        for(int i = 0; i < positive_k_total - 1; i++)
        {
            for(int j = i + 1; j < positive_k_total; j++)
            {
                if(fabs(positive_yellow_lines_array[i][0]) > fabs(positive_yellow_lines_array[j][0]))
                {
                    float temp1 = positive_yellow_lines_array[j][1];
                    float temp2 = positive_yellow_lines_array[j][0];
                    positive_yellow_lines_array[j][1] = positive_yellow_lines_array[i][1];
                    positive_yellow_lines_array[j][0] = positive_yellow_lines_array[i][0];
                    positive_yellow_lines_array[i][1] = temp1;
                    positive_yellow_lines_array[i][0] = temp2;
                }
            }
        }

        /***Calculate Cross Points***/
        int max_points_number = negative_k_total * positive_k_total;
        int cross_points_yellow[max_points_number][2]; //(x,y)

        for(int m = 0; m < negative_k_total; m++)
        {
            for(int n = 0; n < positive_k_total; n++)
            {
                float k1 = negative_yellow_lines_array[m][0];
                float b1 = negative_yellow_lines_array[m][1];
                float k2 = positive_yellow_lines_array[n][0];
                float b2 = positive_yellow_lines_array[n][1];
                //cout<<"("<<negative_yellow_lines_array[m][0]<<","<<b1<<";"<<k2<<","<<b2<<";\n";

                CvPoint2D32f temp_point = lines_cross_point(k1,b1,k2,b2);
                cross_points_yellow[m*positive_k_total+n][0] = (int)temp_point.x;
                cross_points_yellow[m*positive_k_total+n][1] = (int)temp_point.y;
                //cout<<"Point("<<m<<","<<m<<")=("<<cross_points_yellow[m*positive_k_total+n][0]<<","<<cross_points_yellow[m*positive_k_total+n][1]<<")\n";
            }

        }

        /***Find Useful Points***/
        int alfa_point[2];
        int beta_point[2];
        int gamma_point[2];
        int theta_point[2];
        int width_point_delt_x = 0;
        int width_point_delt_y = 0;
        int width_point_delt_x_last = 0;
        int width_point_delt_y_last = 0;
        int length_point_delt_x = 0;
        int length_point_delt_y = 0;
        int length_point_delt_x_last = 0;
        int length_point_delt_y_last = 0;
        float delt_x_scale = 0.2f;
        float delt_y_scale = 0.2f;
        float dist_threshold = 40.f;

        if(!camera_left_side) //right, enemy
        {
            alfa_point[0] = cross_points_yellow[0][0];
            alfa_point[1] = cross_points_yellow[0][1];
            beta_point[0] = cross_points_yellow[positive_k_total][0];
            beta_point[1] = cross_points_yellow[positive_k_total][1];
            gamma_point[0] = cross_points_yellow[1][0];
            gamma_point[1] = cross_points_yellow[1][1];
            theta_point[0] = cross_points_yellow[positive_k_total+1][0];
            theta_point[1] = cross_points_yellow[positive_k_total+1][1];
        }
        else
        {
            alfa_point[0] = cross_points_yellow[0][0];
            alfa_point[1] = cross_points_yellow[0][1];
            beta_point[0] = cross_points_yellow[1][0];
            beta_point[1] = cross_points_yellow[1][1];
            gamma_point[0] = cross_points_yellow[positive_k_total][0];
            gamma_point[1] = cross_points_yellow[positive_k_total][1];
            theta_point[0] = cross_points_yellow[positive_k_total+1][0];
            theta_point[1] = cross_points_yellow[positive_k_total+1][1];
        }

        //first line
        width_point_delt_x = gamma_point[0] - alfa_point[0];
        width_point_delt_y = gamma_point[1] - alfa_point[1];

        cross_points_position_enemy[0][0][0] = alfa_point[0];
        cross_points_position_enemy[0][0][1] = alfa_point[1];

        cvCircle(color_yellow, cvPoint(alfa_point[0], raw_image_area_height - alfa_point[1]), 4, CV_PINK, 6);

        for(int i = 1; i < 5; i++)
        {
            //predict position
            int predict_x = cross_points_position_enemy[0][i-1][0] + (int)(width_point_delt_x * (1 + delt_x_scale));
            int predict_y = cross_points_position_enemy[0][i-1][1] + (int)(width_point_delt_y * (1 + delt_y_scale));
            dist_threshold = (abs(width_point_delt_x) + abs(width_point_delt_y))/4.f;
            int near_points_counter = 0;
            for(int j = 0; j < max_points_number; j++)
            {
                if(point_distance(predict_x, predict_y, cross_points_yellow[j][0], cross_points_yellow[j][1]) < dist_threshold)
                {
                    cross_points_position_enemy[0][i][0] += cross_points_yellow[j][0];
                    cross_points_position_enemy[0][i][1] += cross_points_yellow[j][1];
                    near_points_counter ++;
                    //cout<<"4\n";
                }
            }
            if(near_points_counter > 0)
            {
                cross_points_position_enemy[0][i][0] = cross_points_position_enemy[0][i][0]/near_points_counter;
                cross_points_position_enemy[0][i][1] = cross_points_position_enemy[0][i][1]/near_points_counter;
            }
            else
            {
                cross_points_position_enemy[0][i][0] = predict_x;
                cross_points_position_enemy[0][i][1] = predict_y;
                //cout<<"5\n";
            }

            width_point_delt_x_last = width_point_delt_x;
            width_point_delt_y_last = width_point_delt_y;
            //cout<<"6\n";
            width_point_delt_x = cross_points_position_enemy[0][i][0] - cross_points_position_enemy[0][i-1][0];
            width_point_delt_y = cross_points_position_enemy[0][i][1] - cross_points_position_enemy[0][i-1][1];
            //cout<<"7\n";
            delt_x_scale = fabs((float)width_point_delt_x)/fabs((float)width_point_delt_x_last) - 0.95;
            delt_y_scale = fabs((float)width_point_delt_y)/fabs((float)width_point_delt_y_last) - 0.95;
            //cout<<"a time \n";
        }

        //second line
        width_point_delt_x = theta_point[0] - beta_point[0];
        width_point_delt_y = theta_point[1] - beta_point[1];

        cross_points_position_enemy[1][0][0] = beta_point[0];
        cross_points_position_enemy[1][0][1] = beta_point[1];

        delt_x_scale = 0.2f;
        delt_y_scale = 0.2f;

        for(int i = 1; i < 5; i++)
        {
            //predict position
            int predict_x = cross_points_position_enemy[1][i-1][0] + (int)(width_point_delt_x * (1 + delt_x_scale));
            int predict_y = cross_points_position_enemy[1][i-1][1] + (int)(width_point_delt_y * (1 + delt_y_scale));
            dist_threshold = (abs(width_point_delt_x) + abs(width_point_delt_y))/4.f;
            int near_points_counter = 0;
            for(int j = 0; j < max_points_number; j++)
            {
                if(point_distance(predict_x, predict_y, cross_points_yellow[j][0], cross_points_yellow[j][1]) < dist_threshold)
                {
                    cross_points_position_enemy[1][i][0] += cross_points_yellow[j][0];
                    cross_points_position_enemy[1][i][1] += cross_points_yellow[j][1];
                    near_points_counter ++;
                }
            }
            if(near_points_counter > 0)
            {
                cross_points_position_enemy[1][i][0] = cross_points_position_enemy[1][i][0]/near_points_counter;
                cross_points_position_enemy[1][i][1] = cross_points_position_enemy[1][i][1]/near_points_counter;
            }
            else
            {
                cross_points_position_enemy[1][i][0] = predict_x;
                cross_points_position_enemy[1][i][1] = predict_y;
            }

            width_point_delt_x_last = width_point_delt_x;
            width_point_delt_y_last = width_point_delt_y;
            width_point_delt_x = cross_points_position_enemy[1][i][0] - cross_points_position_enemy[1][i-1][0];
            width_point_delt_y = cross_points_position_enemy[1][i][1] - cross_points_position_enemy[1][i-1][1];
            delt_x_scale = fabs((float)width_point_delt_x)/fabs((float)width_point_delt_x_last) - 0.95;
            delt_y_scale = fabs((float)width_point_delt_y)/fabs((float)width_point_delt_y_last) - 0.95;
            //cout<<"a time \n";
        }


        //other lines
        for(int j = 0; j < 5; j++)
        {
            delt_x_scale = 0.2f;
            delt_y_scale = 0.2f;

            length_point_delt_x = cross_points_position_enemy[1][j][0]- cross_points_position_enemy[0][j][0];
            length_point_delt_y = cross_points_position_enemy[1][j][1]- cross_points_position_enemy[0][j][1];
            //cout<<"length_point_delt="<<length_point_delt_x<<","<<length_point_delt_y<<" * ";

            for(int i = 2; i < 5; i++)
            {
                int predict_x = cross_points_position_enemy[i-1][j][0] + (int)(length_point_delt_x * (1 + delt_x_scale));
                int predict_y = cross_points_position_enemy[i-1][j][1] + (int)(length_point_delt_y * (1 + delt_y_scale));
                //cout<<"("<<i<<","<<j<<")=("<<predict_x<<","<<predict_y<<")    ";

                dist_threshold = (abs(length_point_delt_x) + abs(length_point_delt_y))/2.5f;
                int near_points_counter = 0;
                for(int m = 0; m < max_points_number; m++)
                {
                    if(point_distance(predict_x, predict_y, cross_points_yellow[m][0], cross_points_yellow[m][1]) < dist_threshold)
                    {
                        cross_points_position_enemy[i][j][0] += cross_points_yellow[m][0];
                        cross_points_position_enemy[i][j][1] += cross_points_yellow[m][1];
                        near_points_counter ++;
                    }
                }
                if(near_points_counter > 0)
                {
                    cross_points_position_enemy[i][j][0] = cross_points_position_enemy[i][j][0]/near_points_counter;
                    cross_points_position_enemy[i][j][1] = cross_points_position_enemy[i][j][1]/near_points_counter;
                }
                else
                {
                    cross_points_position_enemy[i][j][0] = predict_x;
                    cross_points_position_enemy[i][j][1] = predict_y;
                }

                length_point_delt_x_last = length_point_delt_x;
                length_point_delt_y_last = length_point_delt_y;
                length_point_delt_x = cross_points_position_enemy[i][j][0] - cross_points_position_enemy[i-1][j][0];
                length_point_delt_y = cross_points_position_enemy[i][j][1] - cross_points_position_enemy[i-1][j][1];
                delt_x_scale = fabs((float)length_point_delt_x)/fabs((float)length_point_delt_x_last) - 0.85;
                delt_y_scale = fabs((float)length_point_delt_y)/fabs((float)length_point_delt_y_last) - 0.85;
                //cout<<"length_point_delt="<<length_point_delt_x<<","<<length_point_delt_y<<" # ";
            }
        }
    }


    /***Draw cross points***/

    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            int x = cross_points_position_enemy[i][j][0];
            int y = raw_image_area_height - cross_points_position_enemy[i][j][1];
            cvCircle(fill_color, cvPoint(x, y), 10, CV_GREEN, 2);
        }

    }


    cvNamedWindow("result");
    cvShowImage("result", fill_color);

    cvWaitKey(0);

    /**Release**/

    cvReleaseMemStorage(&storage_yellow_canny);


    cvReleaseImage(&yellow);
    cvReleaseImage(&yellow_canny);
    cvReleaseImage(&color_yellow);

    cvReleaseMat(&yellow_lines_mat);

    return 0;
}

void Camera::closeCamara()
{
    timer->stop();         // 停止读取数据。
    cvReleaseCapture(&cam);//释放内存；
    bool_open_camera=false;
}


float Camera::point_distance(int x1, int y1, int x2, int y2)
{
    return sqrt((float)(x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

float Camera::point_distance_f(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

bool Camera::f_euqal(float a, float b)
{
    if(fabs(a-b)<0.0001) return true;
    else return false;
}

CvPoint2D32f Camera::lines_cross_point(float k1, float b1, float k2, float b2)
{
    CvPoint2D32f p;
    p.x = (b2 - b1)/(k1 - k2);
    p.y = k1*p.x + b1;

    return p;
}

CvMat* Camera::find_useful_lines_kb(CvSeq* lines_s, int width, int height, int length, float delt_k , float delt_d_n, float delt_d_p)  //return cvmat[0] means total number
{
    int lines_num = lines_s->total;
    float lines[lines_num][2];

    int counter = 0;
    for (int i = 0; i < lines_s->total; i++)
    {
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines_s, i);

        if(point_distance(line[0].x, line[0].y, line[1].x, line[1].y) > length) // by length
        {
            // To normal coordinate
            float x1 = line[0].x;
            float y1 = height-line[0].y;
            float x2 = line[1].x;
            float y2 = height-line[1].y;

            lines[counter][0] = (y2-y1)/(x2-x1);
            lines[counter][1] = y1 - lines[counter][0]*x1;
            counter ++;
        }
        else lines_num --;
    }

    for(int i = 0; i < lines_num - 1; i++)
    {
        for(int j = i+1; j < lines_num; j++)
        {

            if(fabs(lines[i][0]-lines[j][0]) < delt_k)  //fabs(lines[i][1]-lines[j][1])/fabs(lines[j][1]) < delt_b)
            {
                float k1 = (lines[i][0] + lines[j][0])/2.f;
                float k2 = -1.f/k1;
                float b = height/2.f - k2*width/2.f;
                CvPoint2D32f cross_point_1 = lines_cross_point(k1, lines[i][1], k2, b);
                CvPoint2D32f cross_point_2 = lines_cross_point(k1, lines[j][1], k2, b);
                float distance = point_distance_f(cross_point_1.x, cross_point_1.y, cross_point_2.x, cross_point_2.y);

                if(k1 < 0 && distance < delt_d_n)
                {
                    lines[i][0] = k1;
                    lines[i][1] = (lines[i][1] + lines[j][1])/2;
                    //reset sequence
                    for(int h = j+1; h < lines_num; h++)
                    {
                        lines[h-1][0] = lines[h][0];
                        lines[h-1][1] = lines[h][1];
                    }
                    lines_num --;
                    j--;
                }
                else if(k1 > 0 && distance < delt_d_p)
                {
                    lines[i][0] = k1;
                    lines[i][1] = (lines[i][1] + lines[j][1])/2;
                    //reset sequence
                    for(int h = j+1; h < lines_num; h++)
                    {
                        lines[h-1][0] = lines[h][0];
                        lines[h-1][1] = lines[h][1];
                    }
                    lines_num --;
                    j--;
                }
                else ;
            }
        }
    }


    cout<<"lines_num = "<<lines_num<<endl;

    CvMat* lines_mat = cvCreateMat(lines_num+1,2,CV_32FC1);

    CV_MAT_ELEM(*lines_mat, float, 0, 0 ) = (float)lines_num;
    CV_MAT_ELEM(*lines_mat, float, 0, 1 ) = 0.f;

    for(int i=0;i<lines_num;i++)
    {
        CV_MAT_ELEM(*lines_mat, float, i+1, 0 ) = lines[i][0];
        CV_MAT_ELEM(*lines_mat, float, i+1, 1 ) = lines[i][1];
    }

    return lines_mat;
}


void Camera::draw_result_lines(CvMat *lines_mat, IplImage* image, CvScalar color, int thickness)
{
    for(int i = 1; i <= CV_MAT_ELEM(*lines_mat, float, 0, 0 ); i++)
    {
        CvPoint p1;
        CvPoint p2;
        bool p1_set = false;
        //cross point with x = 0 and x = width
        int y1 = CV_MAT_ELEM(*lines_mat, float, i, 1 );
        if(y1 > 0 && y1 < raw_image_area_height) {
            p1.x = 0;
            p1.y = y1;
            p1_set = true;
        }
        int y2 = CV_MAT_ELEM(*lines_mat, float, i, 0 )*raw_image_area_width +CV_MAT_ELEM(*lines_mat, float, i, 1 );
        if(y2 > 0 && y2 < raw_image_area_height) {
            if(p1_set){
                p2.x = raw_image_area_width;
                p2.y = y2;
                //continue;
            }
            else
            {
                p1.x = raw_image_area_width;
                p1.y = y2;
                p1_set = true;
            }
        }
        //cross point with y = 0 and y = height
        int x3 = (0-CV_MAT_ELEM(*lines_mat, float, i, 1 ))/CV_MAT_ELEM(*lines_mat, float, i, 0 );
        if(x3 > 0 && x3 < raw_image_area_width) {
            if(p1_set){
                p2.x = x3;
                p2.y = 0;
                //continue;
            }
            else
            {
                p1.x = x3;
                p1.y = 0;
                p1_set = true;
            }
        }
        int x4 = (raw_image_area_height - CV_MAT_ELEM(*lines_mat, float, i, 1 ))/CV_MAT_ELEM(*lines_mat, float, i, 0 );
        if(x4 > 0 && x4 < raw_image_area_width) {
                p2.x = x4;
                p2.y = raw_image_area_height;
        }

        //translate to image coordinate
        p1.y = raw_image_area_height - p1.y;
        p2.y = raw_image_area_height - p2.y;
        cvLine(image, p1, p2 , color, thickness, CV_AA, 0);
    }
}

CvPoint2D32f Camera::point_slant_coordinate_tranlate(float kx, float ky, float x0, float y0)
{
    CvPoint2D32f x_crossp = lines_cross_point(ky, y0-ky*x0, kx, 0.f);
    CvPoint2D32f y_crossp = lines_cross_point(kx, y0-kx*x0, ky, 0.f);
    CvPoint2D32f result_p;
    result_p.x = point_distance_f(x_crossp.x, x_crossp.y, 0.f, 0.f);
    result_p.y = point_distance_f(y_crossp.x, y_crossp.y, 0.f, 0.f);

    if((x0 - y0 / ky) < 0) result_p.x = 0.f - result_p.x;
    if((x0 - y0 / kx) < 0) result_p.y = 0.f - result_p.y;

    return result_p;
}
