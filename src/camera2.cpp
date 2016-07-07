#include "camera2.h"
#include "stdio.h"
#include <string>
#include <iostream>
#include "receiver.h"

using namespace std;

extern MavrosMessage message;

void onMouse2(int Event,int x,int y,int flags,void* param);
int mouse_click_counter_2 = 0;
int cross_points_position_2[5][5][2];
int cross_points_position_save_2[5][5][2];

float cross_points_real_position_right_enemy_2[5][5][2] = {  //from left up corner, rows and cols
    3.15f, 1.2f, 3.15f, 0.6f, 3.15f, 0.f, 3.15f, -0.6f, 3.15f, -1.2f,
    2.55f, 1.2f, 2.55f, 0.6f, 2.55f, 0.f, 2.55f, -0.6f, 2.55f, -1.2f,
    1.95f, 1.2f, 1.95f, 0.6f, 1.95f, 0.f, 1.95f, -0.6f, 1.95f, -1.2f,
    1.35f, 1.2f, 1.35f, 0.6f, 1.35f, 0.f, 1.35f, -0.6f, 1.35f, -1.2f,
    0.75f, 1.2f, 0.75f, 0.6f, 0.75f, 0.f, 0.75f, -0.6f, 0.75f, -1.2f
};
float cross_points_real_position_right_ours_2[5][5][2] = {  //from left up corner, rows and cols
    -0.75f, 1.2f, -0.75f, 0.6f, -0.75f, 0.f, -0.75f, -0.6f, -0.75f, -1.2f,
    -1.35f, 1.2f, -1.35f, 0.6f, -1.35f, 0.f, -1.35f, -0.6f, -1.35f, -1.2f,
    -1.95f, 1.2f, -1.95f, 0.6f, -1.95f, 0.f, -1.95f, -0.6f, -1.95f, -1.2f,
    -2.55f, 1.2f, -2.55f, 0.6f, -2.55f, 0.f, -2.55f, -0.6f, -2.55f, -1.2f,
    -3.15f, 1.2f, -3.15f, 0.6f, -3.15f, 0.f, -3.15f, -0.6f, -3.15f, -1.2f
};

float cross_points_real_position_left_enemy_2[5][5][2];//from right up corner, rows and cols, auto value
float cross_points_real_position_left_ours_2[5][5][2];//from right up corner, rows and cols, auto value


Camera2::Camera2()
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
    QFile file("/home/chg/catkin_ws/src/ardrone_station/parameters/2_Intrinsic.xml");
    if (file.exists()){
        intrinsic=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/2_Intrinsic.xml");
        load_1 = true;
    }
    else load_1 = false;

    QFile file2("/home/chg/catkin_ws/src/ardrone_station/parameters/2_Distortion.xml");
    if (file2.exists()){
        distortion=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/2_Distortion.xml");
        load_2 = true;
    }
    else load_2 = false;

    if(load_1 && load_2)
    {
        bool_clibration = true;
        intrinsic=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/2_Intrinsic.xml");
        distortion=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/2_Distortion.xml");
    }
    else bool_clibration = false;




    init_paras();

}

Camera2::~Camera2()
{
    delete timer;
}

void Camera2::init_paras()
{

    capture = false;

    bool_cut = false;

    robot_position_updated = false;

    position_clibration_done = false;

    height_threshold = 0;

    neg_d = 20;
    pos_d = 15;

    found_field = false;

    robot_image_p[0] = 0.f;
    robot_image_p[1] = 0.f;
    robot_real_p[0] = 0.f;
    robot_real_p[1] = 0.f;


    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            cross_points_real_position_left_enemy_2[i][j][0] = cross_points_real_position_right_enemy_2[i][4-j][0];
            cross_points_real_position_left_enemy_2[i][j][1] = cross_points_real_position_right_enemy_2[i][4-j][1];
        }
    }

    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            cross_points_real_position_left_ours_2[i][j][0] = cross_points_real_position_right_ours_2[i][4-j][0];
            cross_points_real_position_left_ours_2[i][j][1] = cross_points_real_position_right_ours_2[i][4-j][1];
        }
    }

    read_saved_cross_points();
}

int Camera2::read_saved_cross_points()
{
    char dir_path[80]="/home/chg/catkin_ws/src/ardrone_station/parameters";

    QDir *temp = new QDir;
    bool exist = temp->exists(QString(dir_path));
    if(!exist)
    {
        cout<<"no cross points file found!"<<endl;
        return 0;
    }

    char path1[100];
    strcpy(path1,dir_path);
    char name1[25] = "/camera_position_2.txt";
    strcat(path1,name1);

    fstream config_f1;
    config_f1.open(path1,ios::in);

    int read_counter=0;
    while(!config_f1.eof())
    {   //while not the end of file
        char str[30];
        config_f1 >> str;

        int num = 0;
        int start_place = 0;
        for(int i=0;i<30;i++)
        {
            if(str[i]=='#') { start_place = i; break;}
        }

        for(int j=start_place-1, times=1; j>=0; j--)
        {
            num += (str[j]-'0') * times;
            times *= 10;
        }
        cross_points_position_2[read_counter/10][(read_counter%10)/2][read_counter%2] = num;

        read_counter ++;
    }
    config_f1.close(); //reading finished

    position_clibration_done = true;
    cout<<"cross points file found!"<<"  read_counter="<<read_counter<<endl;

    return 1;
}

bool Camera2::openCamara()
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
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_WIDTH,raw_image_area_width_2);
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_HEIGHT,raw_image_area_height_2);

        timer->start(33);              // 开始计时，超时则发出timeout()信号，30帧/s
        bool_open_camera=true;

        return true;
    }

}


int Camera2::readFarme()
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
            if(i < (float)height_threshold/100.f*raw_image_area_height_2)
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
            s_red = cvGet2D(frame_raw,i,j); // get the (i,j) pixel value
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

    /*cut useless area*/
    if(bool_cut && bool_cut_ticked)
    {
        if(n_b_max > -100000.f && n_b_min < 100000.f && fabs(n_b_max - n_b_min) > 100.f)
        {
            CvScalar s2;

            for(int i = 0;i < fill_color->height;i++)
            {
                for(int j = 0;j < fill_color->width;j++)
                {
                    int x = j;
                    int y = raw_image_area_height_2 - i;
                    if((n_k_max*x-y+n_b_max) < 0 || (n_k_min*x-y+n_b_min) > 0)
                    {
                        s2 = cvGet2D(fill_color,i,j); // get the (i,j) pixel value, rows, cols
                        s2.val[0]=100;
                        s2.val[1]=100;
                        s2.val[2]=100;
                        cvSet2D(fill_color,i,j,s2);//set the (i,j) pixel value
                    }
                }
            }
        }

        if(p_b_max > -100000.f && p_b_min < 100000.f && fabs(p_b_max - p_b_min) > 100.f)
        {
            CvScalar s3;

            for(int i = 0;i < fill_color->height;i++)
            {
                for(int j = 0;j < fill_color->width;j++)
                {
                    int x = j;
                    int y = raw_image_area_height_2 - i;
                    if((p_k_max*x-y+p_b_max) > 0 || (p_k_min*x-y+p_b_min) > 0)
                    {
                        s3 = cvGet2D(fill_color,i,j); // get the (i,j) pixel value, rows, cols
                        s3.val[0]=100;
                        s3.val[1]=100;
                        s3.val[2]=100;
                        cvSet2D(fill_color,i,j,s3);//set the (i,j) pixel value
                    }
                }
            }
        }
    }


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
                    cvSet2D(red,i,j,CV_WHITE_2);//set the (i,j) pixel value
                }
                else
                {
                    cvSet2D(red,i,j,CV_BLACK_2);//set the (i,j) pixel value
                }
            }
        }


        cvErode( red, red, NULL, 1);
        cvDilate( red, red, NULL, 2);

        CvPoint2D32f robot_center;
        float radius;

        CvMemStorage* red_storage = cvCreateMemStorage(0);
        CvSeq* red_contours = 0;
        int contour_num = cvFindContours(red, red_storage, &red_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cout<<"contour_num="<<contour_num<<endl;
        if(contour_num > 0)
        {
            cvDrawContours(fill_color, red_contours, CV_GREEN_2, CV_GREEN_2, 50);

            CvSeq *c = 0;
            bool found = false;
            int counter = 0;
            robot_image_p[0] = 0.f;
            robot_image_p[1] = 0.f;
            for (c = red_contours;c !=NULL;c = c->h_next)
            {
                double area = fabs(cvContourArea(c,CV_WHOLE_SEQ));
                if(area > 100)
                {
                    cvMinEnclosingCircle(c,&robot_center,&radius);
                    cvCircle(fill_color,cvPointFrom32f(robot_center),4,CV_GREEN_2,4);
                    robot_image_p[0] += robot_center.x;
                    robot_image_p[1] += raw_image_area_height_2 - robot_center.y;
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
                    distance_temp[i][j] = point_distance_f(robot_image_p[0], robot_image_p[1], cross_points_position_2[i][j][0], cross_points_position_2[i][j][1]);
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
                CvPoint2D32f min_dist_p1_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, cross_points_position_2[min_row][min_col][0], cross_points_position_2[min_row][min_col][1]);
                CvPoint2D32f min_dist_p2_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, cross_points_position_2[min_row_2][min_col_2][0], cross_points_position_2[min_row_2][min_col_2][1]);
                CvPoint2D32f robot_image_p_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, robot_image_p[0], robot_image_p[1]);

                if(camera_enemy_side)
                {
                    float kx = (cross_points_real_position_right_enemy_2[min_row][min_col][0] - cross_points_real_position_right_enemy_2[min_row_2][min_col_2][0]) / (min_dist_p1_tr.x - min_dist_p2_tr.x);
                    float ky = (cross_points_real_position_right_enemy_2[min_row][min_col][1] - cross_points_real_position_right_enemy_2[min_row_2][min_col_2][1]) / (min_dist_p1_tr.y - min_dist_p2_tr.y);

                    robot_real_p[0] = (cross_points_real_position_right_enemy_2[min_row][min_col][0] + kx*(robot_image_p_tr.x - min_dist_p1_tr.x) + cross_points_real_position_right_enemy_2[min_row_2][min_col_2][0] + kx*(robot_image_p_tr.x - min_dist_p2_tr.x))/2.f;
                    robot_real_p[1] = (cross_points_real_position_right_enemy_2[min_row][min_col][1] + ky*(robot_image_p_tr.y - min_dist_p1_tr.y) + cross_points_real_position_right_enemy_2[min_row_2][min_col_2][1] + ky*(robot_image_p_tr.y - min_dist_p2_tr.y))/2.f;
                }
                else
                {
                    float kx = (cross_points_real_position_right_ours_2[min_row][min_col][0] - cross_points_real_position_right_ours_2[min_row_2][min_col_2][0]) / (min_dist_p1_tr.x - min_dist_p2_tr.x);
                    float ky = (cross_points_real_position_right_ours_2[min_row][min_col][1] - cross_points_real_position_right_ours_2[min_row_2][min_col_2][1]) / (min_dist_p1_tr.y - min_dist_p2_tr.y);

                    robot_real_p[0] = (cross_points_real_position_right_ours_2[min_row][min_col][0] + kx*(robot_image_p_tr.x - min_dist_p1_tr.x) + cross_points_real_position_right_ours_2[min_row_2][min_col_2][0] + kx*(robot_image_p_tr.x - min_dist_p2_tr.x))/2.f;
                    robot_real_p[1] = (cross_points_real_position_right_ours_2[min_row][min_col][1] + ky*(robot_image_p_tr.y - min_dist_p1_tr.y) + cross_points_real_position_right_ours_2[min_row_2][min_col_2][1] + ky*(robot_image_p_tr.y - min_dist_p2_tr.y))/2.f;
                }

            }
            else
            {
                //when left side
                cout<<"negative_k_average "<<negative_k_average<<"positive_k_average"<<positive_k_average<<"cross_points_position_2"<<cross_points_position_2[min_row][min_col][0]<<" , "<<cross_points_position_2[min_row][min_col][1]<<endl;
                CvPoint2D32f min_dist_p1_tr = point_slant_coordinate_tranlate(negative_k_average, positive_k_average, cross_points_position_2[min_row][min_col][0], cross_points_position_2[min_row][min_col][1]);
                CvPoint2D32f min_dist_p2_tr = point_slant_coordinate_tranlate(negative_k_average, positive_k_average, cross_points_position_2[min_row_2][min_col_2][0], cross_points_position_2[min_row_2][min_col_2][1]);
                CvPoint2D32f robot_image_p_tr = point_slant_coordinate_tranlate(negative_k_average, positive_k_average, robot_image_p[0], robot_image_p[1]);

                cout<<"px = "<<min_dist_p1_tr.x<<" , py = "<<min_dist_p1_tr.y<<endl;
                cout<<"rx = "<<robot_image_p_tr.x<<" , ry = "<<robot_image_p_tr.y<<endl;

                if(camera_enemy_side)
                {
                    float kx = (cross_points_real_position_left_enemy_2[min_row][min_col][0] - cross_points_real_position_left_enemy_2[min_row_2][min_col_2][0]) / (min_dist_p1_tr.x - min_dist_p2_tr.x);
                    float ky = (cross_points_real_position_left_enemy_2[min_row][min_col][1] - cross_points_real_position_left_enemy_2[min_row_2][min_col_2][1]) / (min_dist_p1_tr.y - min_dist_p2_tr.y);
                    //cout<<"kx = "<<kx<<" , ky = "<<ky<<endl;

                    robot_real_p[0] = (cross_points_real_position_left_enemy_2[min_row][min_col][0] + kx*(robot_image_p_tr.x - min_dist_p1_tr.x) + cross_points_real_position_left_enemy_2[min_row_2][min_col_2][0] + kx*(robot_image_p_tr.x - min_dist_p2_tr.x))/2.f;
                    robot_real_p[1] = (cross_points_real_position_left_enemy_2[min_row][min_col][1] + ky*(robot_image_p_tr.y - min_dist_p1_tr.y) + cross_points_real_position_left_enemy_2[min_row_2][min_col_2][1] + ky*(robot_image_p_tr.y - min_dist_p2_tr.y))/2.f;
                }
                else
                {
                    float kx = (cross_points_real_position_left_ours_2[min_row][min_col][0] - cross_points_real_position_left_ours_2[min_row_2][min_col_2][0]) / (min_dist_p1_tr.x - min_dist_p2_tr.x);
                    float ky = (cross_points_real_position_left_ours_2[min_row][min_col][1] - cross_points_real_position_left_ours_2[min_row_2][min_col_2][1]) / (min_dist_p1_tr.y - min_dist_p2_tr.y);
                    //cout<<"kx = "<<kx<<" , ky = "<<ky<<endl;

                    robot_real_p[0] = (cross_points_real_position_left_ours_2[min_row][min_col][0] + kx*(robot_image_p_tr.x - min_dist_p1_tr.x) + cross_points_real_position_left_ours_2[min_row_2][min_col_2][0] + kx*(robot_image_p_tr.x - min_dist_p2_tr.x))/2.f;
                    robot_real_p[1] = (cross_points_real_position_left_ours_2[min_row][min_col][1] + ky*(robot_image_p_tr.y - min_dist_p1_tr.y) + cross_points_real_position_left_ours_2[min_row_2][min_col_2][1] + ky*(robot_image_p_tr.y - min_dist_p2_tr.y))/2.f;
                }

            }

            //position offset
            if(camera_enemy_side)
            {
                robot_real_p[0] = robot_real_p[0]*0.852 - 0.1146;
                robot_real_p[1] = robot_real_p[1]*0.9825 - 1.75;
            }
            else {
                robot_real_p[0] = robot_real_p[0]*0.92776 - 0.3441;
                robot_real_p[1] = robot_real_p[1]*0.941 - 0.18;
            }

            robot_position_updated = true;


            //cout<<"Point1 Position = ("<<cross_points_real_position_right_enemy_2[min_row][min_col][0]<<","<<cross_points_real_position_right_enemy_2[min_row][min_col][1]<<")\n";
            //cout<<"Point2 Position = ("<<cross_points_real_position_right_enemy_2[min_row_2][min_col_2][0]<<","<<cross_points_real_position_right_enemy_2[min_row_2][min_col_2][1]<<")\n";
            cout<<"Robot image Position = ("<<robot_image_p[0]<<","<<robot_image_p[1]<<")\n";
            cout<<"Robot Real Position = ("<<robot_real_p[0]<<","<<robot_real_p[1]<<")\n";
            cvCircle(fill_color, cvPoint(cross_points_position_2[min_row][min_col][0], raw_image_area_height_2 - cross_points_position_2[min_row][min_col][1]), 4, CV_GREEN_2, 6);
            cvCircle(fill_color, cvPoint(cross_points_position_2[min_row_2][min_col_2][0], raw_image_area_height_2 - cross_points_position_2[min_row_2][min_col_2][1]), 4, CV_GREEN_2, 6);
            cvCircle(fill_color, cvPoint((int)robot_image_p[0], raw_image_area_height_2 - (int)robot_image_p[1]), 10, CV_RED_2, 2);
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

int Camera2::auto_position()
{
    closeCamara();

    char image_name[100] = "/home/chg/catkin_ws/src/1.jpg";
    IplImage* frame_raw_2 = cvLoadImage(image_name);
    cout<<"Loaded!\n";

    //test codes
    IplImage* frame_raw=cvCreateImage(cvSize(raw_image_area_width_2,raw_image_area_height_2),8,3);//4:3画面
    cvResize(frame_raw_2,frame_raw,CV_INTER_NN);

    CvScalar s_raw;

    for(int i = 0;i < frame_raw->height;i++)
    {
        for(int j = 0;j < frame_raw->width;j++)
        {
            s_raw = cvGet2D(frame_raw,i,j); // get the (i,j) pixel value
            if(i < (float)height_threshold/100.f*raw_image_area_height_2)
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
            s_red = cvGet2D(frame_raw,i,j); // get the (i,j) pixel value
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
                cvSet2D(yellow,i,j,CV_WHITE_2);//set the (i,j) pixel value
            }
            else
            {
                cvSet2D(yellow,i,j,CV_BLACK_2);//set the (i,j) pixel value
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
        cvLine(color_yellow, line[0], line[1], CV_RED_2, 1, CV_AA, 0);
    }


    /***Find the useful lines***/
    CvMat* yellow_lines_mat = find_useful_lines_kb(lines_yellow, raw_image_area_width_2, raw_image_area_height_2, 50, 0.1, neg_d, pos_d);
    draw_result_lines(yellow_lines_mat, fill_color, CV_YELLOW_2);


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
        if(camera_left_side)
        {
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
                    if(positive_yellow_lines_array[i][1] < positive_yellow_lines_array[j][1])
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
        }
        else
        {
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
                    if(positive_yellow_lines_array[i][1] > positive_yellow_lines_array[j][0])
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

        cross_points_position_2[0][0][0] = alfa_point[0];
        cross_points_position_2[0][0][1] = alfa_point[1];

        cvCircle(fill_color, cvPoint(alfa_point[0], raw_image_area_height_2 - alfa_point[1]), 4, CV_PINK_2, 6);

        for(int i = 1; i < 5; i++)
        {
            //predict position
            int predict_x = cross_points_position_2[0][i-1][0] + (int)(width_point_delt_x * (1 + delt_x_scale));
            int predict_y = cross_points_position_2[0][i-1][1] + (int)(width_point_delt_y * (1 + delt_y_scale));

            int near_points_counter = 0;
            float threshold_x = length_point_delt_x / 3.5f;
            float threshold_y = length_point_delt_y / 3.5f;

            for(int j = 0; j < max_points_number; j++)
            {
                if(fabs(predict_x - cross_points_yellow[j][0])< threshold_x&& fabs(predict_y-cross_points_yellow[j][1])<threshold_y)
                {
                    cross_points_position_2[1][i][0] += cross_points_yellow[j][0];
                    cross_points_position_2[1][i][1] += cross_points_yellow[j][1];
                    near_points_counter ++;
                }
            }
            if(near_points_counter > 0)
            {
                cross_points_position_2[0][i][0] = cross_points_position_2[0][i][0]/near_points_counter;
                cross_points_position_2[0][i][1] = cross_points_position_2[0][i][1]/near_points_counter;
            }
            else
            {
                cross_points_position_2[0][i][0] = predict_x;
                cross_points_position_2[0][i][1] = predict_y;
                //cout<<"5\n";
            }

            width_point_delt_x_last = width_point_delt_x;
            width_point_delt_y_last = width_point_delt_y;
            //cout<<"6\n";
            width_point_delt_x = cross_points_position_2[0][i][0] - cross_points_position_2[0][i-1][0];
            width_point_delt_y = cross_points_position_2[0][i][1] - cross_points_position_2[0][i-1][1];
            //cout<<"7\n";
            delt_x_scale = fabs((float)width_point_delt_x)/fabs((float)width_point_delt_x_last) - 1.1f;
            delt_y_scale = fabs((float)width_point_delt_y)/fabs((float)width_point_delt_y_last) - 1.1f;
            //cout<<"a time \n";
        }

        //second line
        width_point_delt_x = theta_point[0] - beta_point[0];
        width_point_delt_y = theta_point[1] - beta_point[1];

        cross_points_position_2[1][0][0] = beta_point[0];
        cross_points_position_2[1][0][1] = beta_point[1];

        delt_x_scale = 0.2f;
        delt_y_scale = 0.2f;

        for(int i = 1; i < 5; i++)
        {
            //predict position
            int predict_x = cross_points_position_2[1][i-1][0] + (int)(width_point_delt_x * (1 + delt_x_scale));
            int predict_y = cross_points_position_2[1][i-1][1] + (int)(width_point_delt_y * (1 + delt_y_scale));

            int near_points_counter = 0;
            float threshold_x = length_point_delt_x / 3.5f;
            float threshold_y = length_point_delt_y / 3.5f;

            for(int j = 0; j < max_points_number; j++)
            {
                if(fabs(predict_x - cross_points_yellow[j][0])< threshold_x&& fabs(predict_y-cross_points_yellow[j][1])<threshold_y)
                {
                    cross_points_position_2[1][i][0] += cross_points_yellow[j][0];
                    cross_points_position_2[1][i][1] += cross_points_yellow[j][1];
                    near_points_counter ++;
                }
            }
            if(near_points_counter > 0)
            {
                cross_points_position_2[1][i][0] = cross_points_position_2[1][i][0]/near_points_counter;
                cross_points_position_2[1][i][1] = cross_points_position_2[1][i][1]/near_points_counter;
            }
            else
            {
                cross_points_position_2[1][i][0] = predict_x;
                cross_points_position_2[1][i][1] = predict_y;
            }

            width_point_delt_x_last = width_point_delt_x;
            width_point_delt_y_last = width_point_delt_y;
            width_point_delt_x = cross_points_position_2[1][i][0] - cross_points_position_2[1][i-1][0];
            width_point_delt_y = cross_points_position_2[1][i][1] - cross_points_position_2[1][i-1][1];
            delt_x_scale = fabs((float)width_point_delt_x)/fabs((float)width_point_delt_x_last) - 1.1f;
            delt_y_scale = fabs((float)width_point_delt_y)/fabs((float)width_point_delt_y_last) - 1.1f;
            //cout<<"a time \n";
        }


        //other lines
        for(int j = 0; j < 5; j++)
        {
            delt_x_scale = 0.2f;
            delt_y_scale = 0.2f;

            length_point_delt_x = cross_points_position_2[1][j][0]- cross_points_position_2[0][j][0];
            length_point_delt_y = cross_points_position_2[1][j][1]- cross_points_position_2[0][j][1];
            //cout<<"length_point_delt="<<length_point_delt_x<<","<<length_point_delt_y<<" * ";

            for(int i = 2; i < 5; i++)
            {
                int predict_x = cross_points_position_2[i-1][j][0] + (int)(length_point_delt_x * (1 + delt_x_scale));
                int predict_y = cross_points_position_2[i-1][j][1] + (int)(length_point_delt_y * (1 + delt_y_scale));
                //cout<<"("<<i<<","<<j<<")=("<<predict_x<<","<<predict_y<<")    ";


                int near_points_counter = 0;
                float threshold_x = length_point_delt_x / 3.f;
                float threshold_y = length_point_delt_y / 3.f;

                for(int m = 0; m < max_points_number; m++)
                {
                    if(fabs(predict_x - cross_points_yellow[m][0])< threshold_x&& fabs(predict_y-cross_points_yellow[m][1])<threshold_y)
                    {
                        cross_points_position_2[i][j][0] += cross_points_yellow[m][0];
                        cross_points_position_2[i][j][1] += cross_points_yellow[m][1];
                        near_points_counter ++;
                    }
                }
                if(near_points_counter > 0)
                {
                    cross_points_position_2[i][j][0] = cross_points_position_2[i][j][0]/near_points_counter;
                    cross_points_position_2[i][j][1] = cross_points_position_2[i][j][1]/near_points_counter;
                }
                else
                {
                    cross_points_position_2[i][j][0] = predict_x;
                    cross_points_position_2[i][j][1] = predict_y;
                }

                length_point_delt_x_last = length_point_delt_x;
                length_point_delt_y_last = length_point_delt_y;
                length_point_delt_x = cross_points_position_2[i][j][0] - cross_points_position_2[i-1][j][0];
                length_point_delt_y = cross_points_position_2[i][j][1] - cross_points_position_2[i-1][j][1];
                delt_x_scale = fabs((float)length_point_delt_x)/fabs((float)length_point_delt_x_last) - 1.1f;
                delt_y_scale = fabs((float)length_point_delt_y)/fabs((float)length_point_delt_y_last) - 1.1f;
                //cout<<"length_point_delt="<<length_point_delt_x<<","<<length_point_delt_y<<" # ";
            }
        }
    }

    /*****Find blue lines ******/
    CvScalar s_blue_black;
    IplImage* blue = cvCreateImage(cvGetSize(fill_color), 8, 1);

    for(int i = 0;i < blue->height;i++)
    {
        for(int j = 0;j < blue->width;j++)
        {
            s_blue_black = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            if(s_blue_black.val[0] == 255 && s_blue_black.val[1] == 144 && s_blue_black.val[2] == 30)
            {
                cvSet2D(blue,i,j,CV_WHITE_2);//set the (i,j) pixel value
            }
            else
            {
                cvSet2D(blue,i,j,CV_BLACK_2);//set the (i,j) pixel value
            }
        }
    }

    cvErode( blue,blue, NULL, 1);
    cvDilate( blue,blue, NULL, 3);

    //find lines
    IplImage* blue_canny = cvCreateImage(cvGetSize(blue), 8, 1);
    cvCanny(blue, blue_canny, 100, 200, 3);

    CvMemStorage* storage_blue_canny = cvCreateMemStorage(0);
    CvSeq* lines_blue = 0;

    IplImage* color_blue = cvCreateImage(cvGetSize(blue_canny), 8, 3);
    cvCvtColor( blue_canny, color_blue, CV_GRAY2BGR );

    lines_blue = cvHoughLines2(blue_canny, storage_blue_canny, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 60, 50, 40);
    cout<<"total lines "<<lines_blue->total<<endl;


    /***Find the useful lines***/
    CvMat* blue_lines_mat = find_useful_lines_kb(lines_blue, raw_image_area_width_2, raw_image_area_height_2, 60, 0.2, 20, 80);


    /***Cut the useless area***/
    n_b_max = -100000.f;  //k is negative, value b when b is max
    n_k_max = 0.f;  //k is negative, value k when b is max
    n_b_min = 100000.f;
    n_k_min = 0.f;
    p_b_max = -100000.f;
    p_k_max = 0.f;
    p_b_min = 100000.f;
    p_k_min = 0.f;

    //find max and min

    for(int i = 1; i < CV_MAT_ELEM(*blue_lines_mat, float, 0, 0) + 1; i++)
    {
        if(CV_MAT_ELEM(*blue_lines_mat, float, i, 0) < 0)
        {
            if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) > n_b_max) {
                n_k_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
                n_b_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
            }
            if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) < n_b_min) {
                n_k_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
                n_b_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
            }
        }
        else
        {
            if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) > n_b_max) {
                p_k_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
                p_b_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
            }
            if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) < n_b_min) {
                p_k_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
                p_b_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
            }
        }
    }

    //cut area
    if(n_b_max > -100000.f && n_b_min < 100000.f && fabs(n_b_max - n_b_min) > 100.f)
    {
        CvScalar s2;

        for(int i = 0;i < fill_color->height;i++)
        {
            for(int j = 0;j < fill_color->width;j++)
            {
                int x = j;
                int y = raw_image_area_height_2 - i;
                if((n_k_max*x-y+n_b_max) < 0 || (n_k_min*x-y+n_b_min) > 0)
                {
                    s2 = cvGet2D(fill_color,i,j); // get the (i,j) pixel value, rows, cols
                    s2.val[0]=100;
                    s2.val[1]=100;
                    s2.val[2]=100;
                    cvSet2D(fill_color,i,j,s2);//set the (i,j) pixel value
                }
            }
        }
    }

    if(p_b_max > -100000.f && p_b_min < 100000.f && fabs(p_b_max - p_b_min) > 100.f)
    {
        CvScalar s3;

        for(int i = 0;i < fill_color->height;i++)
        {
            for(int j = 0;j < fill_color->width;j++)
            {
                int x = j;
                int y = raw_image_area_height_2 - i;
                if((p_k_max*x-y+p_b_max) > 0 || (p_k_min*x-y+p_b_min) > 0)
                {
                    s3 = cvGet2D(fill_color,i,j); // get the (i,j) pixel value, rows, cols
                    s3.val[0]=100;
                    s3.val[1]=100;
                    s3.val[2]=100;
                    cvSet2D(fill_color,i,j,s3);//set the (i,j) pixel value
                }
            }
        }
    }

    draw_result_lines(blue_lines_mat, fill_color, CV_BLUE_2);

    /***Draw cross points***/

    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            int x = cross_points_position_2[i][j][0];
            int y = raw_image_area_height_2 - cross_points_position_2[i][j][1];
            if(i<2) cvCircle(fill_color, cvPoint(x, y), 10, CV_GREEN_2, 2);
            else cvCircle(fill_color, cvPoint(x, y), 10, CV_BROWN_2, 2);
        }

    }


    cvNamedWindow("result");
    cvShowImage("result", fill_color);

    cvWaitKey(2000);

    /**Release**/
    cvDestroyWindow("result");

    cvReleaseMemStorage(&storage_yellow_canny);

    cvReleaseImage(&frame_raw);
    cvReleaseImage(&yellow);
    cvReleaseImage(&yellow_canny);
    cvReleaseImage(&color_yellow);

    cvReleaseMat(&yellow_lines_mat);

    cvReleaseImage(&blue);
    cvReleaseImage(&blue_canny);
    cvReleaseImage(&color_blue);
    cvReleaseMemStorage(&storage_blue_canny);
    cvReleaseMat(&blue_lines_mat);

    openCamara();

    return 0;
}

int Camera2::mannual_position()
{
    closeCamara();

    char image_name[100] = "/home/chg/catkin_ws/src/1.jpg";
    IplImage* frame_raw_2 = cvLoadImage(image_name);
    cout<<"Loaded!\n";

    //test codes
    IplImage* frame_raw=cvCreateImage(cvSize(raw_image_area_width_2,raw_image_area_height_2),8,3);//4:3画面
    cvResize(frame_raw_2,frame_raw,CV_INTER_NN);

    cvNamedWindow("MannualClibration",1);
    cvSetMouseCallback("MannualClibration",onMouse2,(void*) frame_raw);

    IplImage* frame_circles = cvCloneImage(frame_raw);
    while(1)
    {
        cvCopyImage(frame_raw,frame_circles);
        for(int i = 0;i < mouse_click_counter_2;i++)
        {
            int x = cross_points_position_2[i/5][i%5][0];
            int y = raw_image_area_height_2 - cross_points_position_2[i/5][i%5][1];
            cvCircle(frame_circles, cvPoint(x,y),6,CV_WHITE_2,2);
        }
        cvShowImage("MannualClibration",frame_circles);


        if(cvWaitKey(30)==27) break;
        if(mouse_click_counter_2 == 25) break;
    }

    char dir_path[80]="/home/chg/catkin_ws/src/ardrone_station/parameters";

    QDir *temp = new QDir;
    bool exist = temp->exists(QString(dir_path));
    if(!exist)temp->mkdir(QString(dir_path));

    //parameters for camera 2
    char path1[100];
    strcpy(path1,dir_path);
    char name1[25] = "/camera_position_2.txt";
    strcat(path1,name1);

    FILE *pTxtFile1 = NULL;

    pTxtFile1 = fopen(path1, "w+");
    if (pTxtFile1 == NULL)
    {
        printf("The program exist!\n");
        return 0;
    }

    for(int i=0;i<5;i++)
    {
        for(int j=0;j<5;j++)
        {
            fprintf(pTxtFile1,"%d#\n",cross_points_position_save_2[i][j][0]);
            fprintf(pTxtFile1,"%d#\n",cross_points_position_save_2[i][j][1]);
        }
    }

    fclose(pTxtFile1);

    cvDestroyWindow("MannualClibration");
    cvReleaseImage(&frame_circles);
    cvReleaseImage(&frame_raw);
    cvReleaseImage(&frame_raw_2);

    position_clibration_done = true;

    openCamara();

    return 0;
}

void Camera2::closeCamara()
{
    timer->stop();         // 停止读取数据。
    cvReleaseCapture(&cam);//释放内存；
    bool_open_camera=false;
}


float Camera2::point_distance(int x1, int y1, int x2, int y2)
{
    return sqrt((float)(x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

float Camera2::point_distance_f(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

bool Camera2::f_euqal(float a, float b)
{
    if(fabs(a-b)<0.0001) return true;
    else return false;
}

CvPoint2D32f Camera2::lines_cross_point(float k1, float b1, float k2, float b2)
{
    CvPoint2D32f p;
    p.x = (b2 - b1)/(k1 - k2);
    p.y = k1*p.x + b1;

    return p;
}

CvMat* Camera2::find_useful_lines_kb(CvSeq* lines_s, int width, int height, int length, float delt_k , float delt_d_n, float delt_d_p)  //return cvmat[0] means total number
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


void Camera2::draw_result_lines(CvMat *lines_mat, IplImage* image, CvScalar color, int thickness)
{
    for(int i = 1; i <= CV_MAT_ELEM(*lines_mat, float, 0, 0 ); i++)
    {
        CvPoint p1;
        CvPoint p2;
        bool p1_set = false;
        //cross point with x = 0 and x = width
        int y1 = CV_MAT_ELEM(*lines_mat, float, i, 1 );
        if(y1 > 0 && y1 < raw_image_area_height_2) {
            p1.x = 0;
            p1.y = y1;
            p1_set = true;
        }
        int y2 = CV_MAT_ELEM(*lines_mat, float, i, 0 )*raw_image_area_width_2 +CV_MAT_ELEM(*lines_mat, float, i, 1 );
        if(y2 > 0 && y2 < raw_image_area_height_2) {
            if(p1_set){
                p2.x = raw_image_area_width_2;
                p2.y = y2;
                //continue;
            }
            else
            {
                p1.x = raw_image_area_width_2;
                p1.y = y2;
                p1_set = true;
            }
        }
        //cross point with y = 0 and y = height
        int x3 = (0-CV_MAT_ELEM(*lines_mat, float, i, 1 ))/CV_MAT_ELEM(*lines_mat, float, i, 0 );
        if(x3 > 0 && x3 < raw_image_area_width_2) {
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
        int x4 = (raw_image_area_height_2 - CV_MAT_ELEM(*lines_mat, float, i, 1 ))/CV_MAT_ELEM(*lines_mat, float, i, 0 );
        if(x4 > 0 && x4 < raw_image_area_width_2) {
                p2.x = x4;
                p2.y = raw_image_area_height_2;
        }

        //translate to image coordinate
        p1.y = raw_image_area_height_2 - p1.y;
        p2.y = raw_image_area_height_2 - p2.y;
        cvLine(image, p1, p2 , color, thickness, CV_AA, 0);
    }
}

CvPoint2D32f Camera2::point_slant_coordinate_tranlate(float kx, float ky, float x0, float y0)
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

void onMouse2(int Event,int x,int y,int flags,void* param)
{
    if(Event == CV_EVENT_LBUTTONDOWN)
    {
        cout<<"position"<<x<<","<<y<<"   ";
        if(mouse_click_counter_2 < 25) mouse_click_counter_2 ++;
        cout<<mouse_click_counter_2<<"/25"<<endl;
        int i = (mouse_click_counter_2-1)/5;
        int j = (mouse_click_counter_2-1)%5;
        cross_points_position_2[i][j][0] = x;
        cross_points_position_2[i][j][1] = raw_image_area_height_2 - y;
        cross_points_position_save_2[i][j][0] = x;
        cross_points_position_save_2[i][j][1] = raw_image_area_height_2 - y;
    }
    else if(Event == CV_EVENT_RBUTTONDOWN)
    {
        if(mouse_click_counter_2 > 0)mouse_click_counter_2 --;
    }
    else;
}
