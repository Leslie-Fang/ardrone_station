//***********camera_calibration.cpp***********

#include "camera_calibration.h"
#include "stdio.h"
#include <string>
#include <iostream>

using namespace std;

Camera_Calibration::Camera_Calibration()
{
    cam = NULL;
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(readFarme()));


    bool_show_Image=true;
    bool_image_capture = false;
    bool_calibration_done = false;
    camera_seq = 0;

    image_counter = 0;
    total_number = 0;

    line_thickness=2;
    yellow=cvScalar(0,255,255);


    //测试像素代码
    //cvNamedWindow("TEST");
}

Camera_Calibration::~Camera_Calibration()
{
    delete timer;
}

void Camera_Calibration::run()
{
    //openCamara();
    //readFarme();
}

bool Camera_Calibration::openCamara()
{
    cam = cvCreateCameraCapture(camera_seq);//打开摄像头，从摄像头中获取视频
    if(cam==0)
    {
        bool_open_camera=false;
        return false;
    }
    else
    {
        //设定捕获图像大小及帧率
        cvSetCaptureProperty(cam,CV_CAP_PROP_FPS,30);
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_WIDTH,1280);
        cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_HEIGHT,720);

        timer->start(33);              // 开始计时，超时则发出timeout()信号，30帧/s
        bool_open_camera=true;

        image_counter = 0;
        total_number = 0;

        return true;
    }
}

void Camera_Calibration::readFarme()
{
    frame_raw = cvQueryFrame(cam);// 从摄像头中抓取并返回每一帧

    if(bool_calibration_done) //消除畸变
    {
        cvRemap(frame_raw,frame_raw,mapx,mapy);
    }

    //窗口大小适应
    frame=cvCreateImage(cvSize(calibration_image_area_width,calibration_image_area_height),8,3);//4:3画面
    cvResize(frame_raw,frame,CV_INTER_NN);

     // 将抓取到的帧，转换为QImage格式。QImage::Format_RGB888不同的摄像头用不同的格式。
    image = QImage((const uchar*)frame->imageData, frame->width, frame->height,QImage::Format_RGB888).rgbSwapped();

    if(bool_show_Image)camera_Send_Image();

    if(bool_image_capture)
    {
        //储存最大100幅临时截图
        image_temp[image_counter] = QImage((const uchar*)frame_raw->imageData, frame_raw->width, frame_raw->height,QImage::Format_RGB888).rgbSwapped();

        image_counter += 1;
        if(image_counter==100)image_counter=0;

        if(total_number < 100) total_number += 1; //total_number Max: 99, will subtract 1
        bool_image_capture=false;

        camera_Save_Image();

    }


    cvReleaseImage(&frame);//释放图像占用的内存
}

void Camera_Calibration::closeCamara()
{
    timer->stop();         // 停止读取数据。
    cvReleaseCapture(&cam);//释放内存；
    bool_open_camera = false;
    bool_calibration_done = false;

    //测试代码
    //cvDestroyWindow("TEST");
}

void Camera_Calibration::takingPictures()
{
    bool_image_capture=true;
}

void Camera_Calibration::calibration()
{
    board_size = cvSize(cube_length,cube_width);
    int board_width = board_size.width;
    int board_height = board_size.height;
    int total_per_image = board_width*board_height;

    CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];
    CvMat * image_points=cvCreateMat(total_number * total_per_image,2,CV_32FC1);
    CvMat * object_points=cvCreateMat(total_number * total_per_image,3,CV_32FC1);
    CvMat * point_counts=cvCreateMat(total_number,1,CV_32SC1);
    CvMat * intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
    CvMat * distortion_coeffs=cvCreateMat(5,1,CV_32FC1);

    int count;
    int found;
    int step;
    int successes=0;

    IplImage * show;

    total_number -= 1;

    for(int a=0; a<=total_number;a++)
    {
        show = QImageToIplImage(&image_temp[a]);

        found=cvFindChessboardCorners(show,board_size,image_points_buf,&count, CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
        if(found==0)
        {
            cout<<"第"<<a+1<<"帧图片无法找到棋盘格所有角点!\n\n";
            cvNamedWindow("RePlay",1);
            cvShowImage("RePlay",show);
            cvWaitKey(0);
        }
        else
        {
            cout<<"第"<<a+1<<"帧图像成功获得"<<count<<"个角点...\n";

            cvNamedWindow("RePlay",1);

            IplImage * gray_image= cvCreateImage(cvGetSize(show),8,1);
            cvCvtColor(show,gray_image,CV_BGR2GRAY);
            cout<<"获取源图像灰度图过程完成...\n";
            cvFindCornerSubPix(gray_image,image_points_buf,count,cvSize(11,11),cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
            cout<<"灰度图亚像素化过程完成...\n";
            cvDrawChessboardCorners(show,board_size,image_points_buf,count,found);
            cout<<"在源图像上绘制角点过程完成...\n\n";
            cvShowImage("RePlay",show);

            cvWaitKey(0);
        }

        if(total_per_image==count) //succeeded image process
        {
            step=successes*total_per_image;
            for(int i=step,j=0;j<total_per_image;++i,++j)
            {
                CV_MAT_ELEM(*image_points,float,i,0)=image_points_buf[j].x;
                CV_MAT_ELEM(*image_points,float,i,1)=image_points_buf[j].y;
                CV_MAT_ELEM(*object_points,float,i,0)=(float)(j/cube_length);//rows
                CV_MAT_ELEM(*object_points,float,i,1)=(float)(j%cube_length);//cols
                CV_MAT_ELEM(*object_points,float,i,2)=0.0f;
            }
            CV_MAT_ELEM(*point_counts,int,successes,0)=total_per_image;
            successes++;
        }
    }

    cvReleaseImage(&show);
    cvDestroyWindow("RePlay");

    cout<<"*********************************************\n";
    cout<<total_number+1<<"帧图片中，标定失败的图片为"<<total_number+1-successes<<"帧...\n\n";
    cout<<"*********************************************\n\n";

    cout<<"开始计算摄像机内参数...\n";


    CvMat * object_points2=cvCreateMat(successes*total_per_image,3,CV_32FC1);
    CvMat * image_points2=cvCreateMat(successes*total_per_image,2,CV_32FC1);
    CvMat * point_counts2=cvCreateMat(successes,1,CV_32SC1);

    for(int i=0;i<successes*total_per_image;++i)
    {
        CV_MAT_ELEM(*image_points2,float,i,0)=CV_MAT_ELEM(*image_points,float,i,0);
        CV_MAT_ELEM(*image_points2,float,i,1)=CV_MAT_ELEM(*image_points,float,i,1);
        CV_MAT_ELEM(*object_points2,float,i,0)=CV_MAT_ELEM(*object_points,float,i,0);
        CV_MAT_ELEM(*object_points2,float,i,1)=CV_MAT_ELEM(*object_points,float,i,1);
        CV_MAT_ELEM(*object_points2,float,i,2)=CV_MAT_ELEM(*object_points,float,i,2);
    }

    for(int i=0;i<successes;++i)
    {
        CV_MAT_ELEM(*point_counts2,int,i,0)=CV_MAT_ELEM(*point_counts,int,i,0);
    }

    cvReleaseMat(&object_points);
    cvReleaseMat(&image_points);
    cvReleaseMat(&point_counts);

    CV_MAT_ELEM(*intrinsic_matrix,float,0,0)=1.0f;
    CV_MAT_ELEM(*intrinsic_matrix,float,1,1)=1.0f;

    cvCalibrateCamera2(object_points2,image_points2,point_counts2,cvGetSize(frame_raw),
        intrinsic_matrix,distortion_coeffs,NULL,NULL,0);

    cout<<"摄像机内参数矩阵为：\n";
    cout<<CV_MAT_ELEM(*intrinsic_matrix,float,0,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,0,1)
        <<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,0,2)
        <<"\n\n";
    cout<<CV_MAT_ELEM(*intrinsic_matrix,float,1,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,1,1)
        <<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,1,2)
        <<"\n\n";
    cout<<CV_MAT_ELEM(*intrinsic_matrix,float,2,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,2,1)
        <<"          "<<CV_MAT_ELEM(*intrinsic_matrix,float,2,2)
        <<"\n\n";

    cout<<"畸变系数矩阵为：\n";
    cout<<CV_MAT_ELEM(*distortion_coeffs,float,0,0)<<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,1,0)
        <<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,2,0)
        <<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,3,0)
        <<"    "<<CV_MAT_ELEM(*distortion_coeffs,float,4,0)
        <<"\n\n";

    cvSave("/home/chg/catkin_ws/src/ardrone_station/parameters/Intrinsics.xml",intrinsic_matrix);
    cvSave("/home/chg/catkin_ws/src/ardrone_station/parameters/Distortion.xml",distortion_coeffs);

    intrinsic=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/Intrinsics.xml");
    distortion=(CvMat *)cvLoad("/home/chg/catkin_ws/src/ardrone_station/parameters/Distortion.xml");

    mapx=cvCreateImage(cvGetSize(frame_raw),IPL_DEPTH_32F,1);
    mapy=cvCreateImage(cvGetSize(frame_raw),IPL_DEPTH_32F,1);
    cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

    bool_calibration_done = true;

    cout<<"摄像机矩阵、畸变系数向量已经分别存储在名为Intrinsics.xml、Distortion.xml文档中\n\n";

}

IplImage* Camera_Calibration::QImageToIplImage(QImage * qImage)
{
    int width = qImage->width();
    int height = qImage->height();
    CvSize Size;
    Size.height = height;
    Size.width = width;
    IplImage *IplImageBuffer = cvCreateImage(Size, IPL_DEPTH_8U, 3);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            QRgb rgb = qImage->pixel(x, y);
            cvSet2D(IplImageBuffer, y, x, CV_RGB(qRed(rgb), qGreen(rgb), qBlue(rgb)));
        }
    }
    return IplImageBuffer;
}
