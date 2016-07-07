//********mianwindow.cpp*********

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "receiver.h"
#include "QDesktopWidget"
#include "camera.h"
#include "camera2.h"
#include "camera_calibration.h"

extern MavrosMessage message;
extern Camera camera_video;
extern Camera2 camera2_video;
extern Camera_Calibration camera_calibration;

char dir_path[80]="/home/chg/catkin_ws/src/ardrone_station/parameters";

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    QDesktopWidget *d = QApplication::desktop();//屏幕大小捕获
    int w = d->width();     // 返回桌面宽度
    int h = d->height();    // 返回桌面高度
    setMaximumSize(w-60,h-40);
    setMinimumSize(w-60,h-40);

    //定时器
    QTimer *timer=new QTimer(this);
    timer->start(1000);//每秒更新一次

    //信号与槽连接
    QObject::connect(timer,SIGNAL(timeout()),this,SLOT(time_Update()));
    //QObject::connect(&message,SIGNAL(state_Mode_Signal()),this,SLOT(state_Mode_Slot()));

    QObject::connect(&camera_video,SIGNAL(camera_Image_Signal()),this,SLOT(camera_Image_Slot()));
    QObject::connect(&camera2_video,SIGNAL(camera_Image_Signal()),this,SLOT(camera2_Image_Slot()));
    QObject::connect(&camera_calibration,SIGNAL(camera_Image_Signal()),this,SLOT(camera_calibration_Image_Slot()));
    QObject::connect(&camera_calibration,SIGNAL(image_Save_Signal()),this,SLOT(calibration_Save_Image_Slot()));

    init_paras();

    //set cout precision
    cout<<fixed;
    cout.precision(8);


    ui->tabWidget->setCurrentIndex(0);

    //pitch、roll、yaw绘图仪表初始化
    StatusPainter *painter = new StatusPainter();
    ui->tabWidget_PaintArea->addTab(painter,"仪表显示");//添加新的画图区widget new StatusPainter(QColor(141,238,238))
    //if(choice==0)ui->tabWidget_PaintArea->removeTab(0);
    ui->tabWidget_PaintArea->setCurrentIndex(1);//显示第二页
    get_Painter_Address(painter);

    //字体字号颜色设定
    //QFont font1("宋体",12,QFont::Bold);
    //ui->label_Mode->setFont(font1);


    //飞行路径图背景设置

    ui->frame_Fly_Position->setFrameStyle(1);
    ui->frame_Fly_Position->setFixedSize(FLY_POSITION_LABEL_WIDTH,FLY_POSITION_LABEL_HEIGHT);

    QPalette   palette4;
    QPixmap pixmap4(":/icon/Icons/grass-360x270.jpg");//背景图片
    palette4.setBrush(ui->frame_Fly_Position-> backgroundRole(),QBrush(pixmap4));
    ui->frame_Fly_Position->setPalette(palette4);
    ui->frame_Fly_Position->setMask(pixmap4.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Fly_Position->setAutoFillBackground(true);

    fly_position_label = new QLabel(ui->frame_Fly_Position);
    fly_position_label->setFixedWidth(FLY_POSITION_LABEL_WIDTH);
    fly_position_label->setFixedHeight(FLY_POSITION_LABEL_HEIGHT);
    fly_position_label->move(0,0);

    ui->comboBox_Cube_Length->setCurrentIndex(4);

    ui->pushButton_Open_Video_Calibration->setEnabled(true);
    ui->pushButton_Close_Video_Calibration->setEnabled(false);
    ui->pushButton_Video_Calibration_Save->setEnabled(false);
    ui->pushButton_Video_Calibration_Start->setEnabled(false);

    ui->pushButton_Open_Video->setEnabled(true);
    ui->pushButton_Close_Video->setEnabled(false);

    ui->pushButton_Open_Video_2->setEnabled(true);
    ui->pushButton_Close_Video_2->setEnabled(false);

    ui->pushButton_Auto_Position->setEnabled(false);
    ui->pushButton_Auto_Position_2->setEnabled(false);
    ui->pushButton_Mannual_Position->setEnabled(false);
    ui->pushButton_Mannual_Position_2->setEnabled(false);



    //设置是否可用
    //ui->progressBar_GPS->setRange(0,15);
    //ui->progressBar_Battery->setRange(190,245);

    //set conection display
    //ui->label_Controller->setStyleSheet("background-color:red");
    //ui->label_Computer->setStyleSheet("background-color:red");

}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init_paras()
{
    read_saved_paras();

    camera_video.camera_number = 1;
    camera2_video.camera_number = 2;

    ui->horizontalSlider_H_Min_Blue->setValue(camera_video.color_threshold[0][0]);
    ui->horizontalSlider_H_Max_Blue->setValue(camera_video.color_threshold[0][1]);
    ui->horizontalSlider_S_Min_Blue->setValue(camera_video.color_threshold[0][2]);
    ui->horizontalSlider_S_Max_Blue->setValue(camera_video.color_threshold[0][3]);
    ui->horizontalSlider_V_Min_Blue->setValue(camera_video.color_threshold[0][4]);
    ui->horizontalSlider_V_Max_Blue->setValue(camera_video.color_threshold[0][5]);

    ui->horizontalSlider_H_Min_Yellow->setValue(camera_video.color_threshold[1][0]);
    ui->horizontalSlider_H_Max_Yellow->setValue(camera_video.color_threshold[1][1]);
    ui->horizontalSlider_S_Min_Yellow->setValue(camera_video.color_threshold[1][2]);
    ui->horizontalSlider_S_Max_Yellow->setValue(camera_video.color_threshold[1][3]);
    ui->horizontalSlider_V_Min_Yellow->setValue(camera_video.color_threshold[1][4]);
    ui->horizontalSlider_V_Max_Yellow->setValue(camera_video.color_threshold[1][5]);

    ui->horizontalSlider_H_Min_Red->setValue(camera_video.color_threshold[2][0]);
    ui->horizontalSlider_H_Max_Red->setValue(camera_video.color_threshold[2][1]);
    ui->horizontalSlider_S_Min_Red->setValue(camera_video.color_threshold[2][2]);
    ui->horizontalSlider_S_Max_Red->setValue(camera_video.color_threshold[2][3]);
    ui->horizontalSlider_V_Min_Red->setValue(camera_video.color_threshold[2][4]);
    ui->horizontalSlider_V_Max_Red->setValue(camera_video.color_threshold[2][5]);

    ui->horizontalSlider_H_Min_White->setValue(camera_video.color_threshold[3][0]);
    ui->horizontalSlider_H_Max_White->setValue(camera_video.color_threshold[3][1]);
    ui->horizontalSlider_S_Min_White->setValue(camera_video.color_threshold[3][2]);
    ui->horizontalSlider_S_Max_White->setValue(camera_video.color_threshold[3][3]);
    ui->horizontalSlider_V_Min_White->setValue(camera_video.color_threshold[3][4]);
    ui->horizontalSlider_V_Max_White->setValue(camera_video.color_threshold[3][5]);

    ui->horizontalSlider_H_Min_Black->setValue(camera_video.color_threshold[4][0]);
    ui->horizontalSlider_H_Max_Black->setValue(camera_video.color_threshold[4][1]);
    ui->horizontalSlider_S_Min_Black->setValue(camera_video.color_threshold[4][2]);
    ui->horizontalSlider_S_Max_Black->setValue(camera_video.color_threshold[4][3]);
    ui->horizontalSlider_V_Min_Black->setValue(camera_video.color_threshold[4][4]);
    ui->horizontalSlider_V_Max_Black->setValue(camera_video.color_threshold[4][5]);

    ui->horizontalSlider_H_Min_Blue_2->setValue(camera2_video.color_threshold[0][0]);
    ui->horizontalSlider_H_Max_Blue_2->setValue(camera2_video.color_threshold[0][1]);
    ui->horizontalSlider_S_Min_Blue_2->setValue(camera2_video.color_threshold[0][2]);
    ui->horizontalSlider_S_Max_Blue_2->setValue(camera2_video.color_threshold[0][3]);
    ui->horizontalSlider_V_Min_Blue_2->setValue(camera2_video.color_threshold[0][4]);
    ui->horizontalSlider_V_Max_Blue_2->setValue(camera2_video.color_threshold[0][5]);

    ui->horizontalSlider_H_Min_Yellow_2->setValue(camera2_video.color_threshold[1][0]);
    ui->horizontalSlider_H_Max_Yellow_2->setValue(camera2_video.color_threshold[1][1]);
    ui->horizontalSlider_S_Min_Yellow_2->setValue(camera2_video.color_threshold[1][2]);
    ui->horizontalSlider_S_Max_Yellow_2->setValue(camera2_video.color_threshold[1][3]);
    ui->horizontalSlider_V_Min_Yellow_2->setValue(camera2_video.color_threshold[1][4]);
    ui->horizontalSlider_V_Max_Yellow_2->setValue(camera2_video.color_threshold[1][5]);

    ui->horizontalSlider_H_Min_Red_2->setValue(camera2_video.color_threshold[2][0]);
    ui->horizontalSlider_H_Max_Red_2->setValue(camera2_video.color_threshold[2][1]);
    ui->horizontalSlider_S_Min_Red_2->setValue(camera2_video.color_threshold[2][2]);
    ui->horizontalSlider_S_Max_Red_2->setValue(camera2_video.color_threshold[2][3]);
    ui->horizontalSlider_V_Min_Red_2->setValue(camera2_video.color_threshold[2][4]);
    ui->horizontalSlider_V_Max_Red_2->setValue(camera2_video.color_threshold[2][5]);

    ui->horizontalSlider_H_Min_White_2->setValue(camera2_video.color_threshold[3][0]);
    ui->horizontalSlider_H_Max_White_2->setValue(camera2_video.color_threshold[3][1]);
    ui->horizontalSlider_S_Min_White_2->setValue(camera2_video.color_threshold[3][2]);
    ui->horizontalSlider_S_Max_White_2->setValue(camera2_video.color_threshold[3][3]);
    ui->horizontalSlider_V_Min_White_2->setValue(camera2_video.color_threshold[3][4]);
    ui->horizontalSlider_V_Max_White_2->setValue(camera2_video.color_threshold[3][5]);

    ui->horizontalSlider_H_Min_Black_2->setValue(camera2_video.color_threshold[4][0]);
    ui->horizontalSlider_H_Max_Black_2->setValue(camera2_video.color_threshold[4][1]);
    ui->horizontalSlider_S_Min_Black_2->setValue(camera2_video.color_threshold[4][2]);
    ui->horizontalSlider_S_Max_Black_2->setValue(camera2_video.color_threshold[4][3]);
    ui->horizontalSlider_V_Min_Black_2->setValue(camera2_video.color_threshold[4][4]);
    ui->horizontalSlider_V_Max_Black_2->setValue(camera2_video.color_threshold[4][5]);

    ui->radioButton_left_1->setChecked(true);
    ui->radioButton_right_2->setChecked(true);

    ui->radioButton_enemy_1->setChecked(true);
    ui->radioButton_ours_2->setChecked(true);

    camera_video.camera_left_side = true;
    camera_video.camera_enemy_side = true;
    camera2_video.camera_left_side = false;
    camera2_video.camera_enemy_side = false;
}

int MainWindow::read_saved_paras()
{
    QDir *temp = new QDir;
    bool exist = temp->exists(QString(dir_path));
    if(!exist)
    {
        cout<<"no config file found!"<<endl;
        return 0;
    }

    char path1[100];
    strcpy(path1,dir_path);
    char name1[17] = "/camera_1.txt";
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
        camera_video.color_threshold[read_counter/6][read_counter%6] = num;
        //cout<<camera_video.color_threshold[read_counter/6][read_counter%6]<<endl;

        read_counter ++;
    }
    config_f1.close(); //reading finished

    char path2[100];
    strcpy(path2,dir_path);
    char name2[17] = "/camera_2.txt";
    strcat(path2,name2);

    fstream config_f2;
    config_f2.open(path2,ios::in);

    int read_counter2=0;
    while(!config_f2.eof())
    {   //while not the end of file
        char str[30];
        config_f2 >> str;

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
        camera2_video.color_threshold[read_counter2/6][read_counter2%6] = num;
        //cout<<camera_video.color_threshold[read_counter/6][read_counter%6]<<endl;

        read_counter2 ++;
    }
    config_f2.close(); //reading finished

    cout<<"parameter reading finished!"<<endl;

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton button;
    button = QMessageBox::question(this, tr("退出程序"),
                                   QString(tr("确认退出程序?")),
                                   QMessageBox::Yes | QMessageBox::No);
    if (button == QMessageBox::No) {
          event->ignore();  //忽略退出信号，程序继续运行
    }
    else if (button == QMessageBox::Yes) {
        event->accept();  //接受退出信号，程序退出
    }
}

/****显示消息槽****/

void MainWindow::local_Position_Slot()
{
    //去抖动
    if(fabs(message.local_position.orientation.pitchd+message.local_position.orientation.rolld+message.local_position.orientation.yawd-orientation_last)>=2.0)
    {
        status_painter->pitchd=message.local_position.orientation.pitchd;
        status_painter->rolld=message.local_position.orientation.rolld;
        status_painter->pitch=message.local_position.orientation.pitch;
        status_painter->roll=message.local_position.orientation.roll;
        status_painter->compassd=-message.local_position.orientation.yawd+180;
        status_painter->update();
    }

    ui->lineEdit_Orientation_Pitchd->setText(QString::number(message.local_position.orientation.pitchd));
    ui->lineEdit_Orientation_Rolld->setText(QString::number(message.local_position.orientation.rolld));
    ui->lineEdit_Orientation_Yawd->setText(QString::number(message.local_position.orientation.yawd));

    //竖直速度显示，这里的local_position.消息为速度
    ui->lineEdit_Velocity_X->setText(QString::number(message.local_position.speed.x));
    ui->lineEdit_Velocity_Y->setText(QString::number(message.local_position.speed.y));
    ui->lineEdit_Velocity_Z->setText(QString::number(message.local_position.speed.z));

}


void MainWindow::paintEvent(QPaintEvent *event) /*****主界面绘图槽******/
{
    //QPainter mainwindow_painter(this);
    //mainwindow_painter.drawLine(QPoint(0,0),QPoint(100,100));
}

void MainWindow::get_Painter_Address(StatusPainter *painter)
{
    status_painter=painter;
}

void MainWindow::time_Update()
{
    system_time=QTime::currentTime();
    char time_temp[20];
    sprintf(time_temp,"%d:%d:%d",system_time.hour(),system_time.minute(),system_time.second());
    ui->label_System_Time->setText(time_temp);
}

void MainWindow::timer_Slot()
{
    ;
}


float MainWindow::point_dist(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float MainWindow::point_line_dist(float m, float n, float k, float b)
{
    return (fabs(k*m-n+b))/(sqrt(k*k+1));
}


int MainWindow::on_pushButton_Save_Config_clicked()
{

    cout<<"saving...\n";

    QDir *temp = new QDir;
    bool exist = temp->exists(QString(dir_path));
    if(!exist)temp->mkdir(QString(dir_path));

    //parameters for camera 1
    char path1[100];
    strcpy(path1,dir_path);
    char name1[17] = "/camera_1.txt";
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
        for(int j=0;j<6;j++)
        {
            fprintf(pTxtFile1,"%d#\n",camera_video.color_threshold[i][j]);
        }
    }

    fclose(pTxtFile1);

    //parameters for camera 2
    char path2[100];
    strcpy(path2,dir_path);
    char name2[17] = "/camera_2.txt";
    strcat(path2,name2);

    FILE *pTxtFile2 = NULL;

    pTxtFile2 = fopen(path2, "w+");
    if (pTxtFile2 == NULL)
    {
        printf("The program exist!\n");
        return 0;
    }

    for(int i=0;i<5;i++)
    {
        for(int j=0;j<6;j++)
        {
            fprintf(pTxtFile2,"%d#\n",camera2_video.color_threshold[i][j]);
        }
    }

    fclose(pTxtFile2);

    /*
    char path2[100];
    strcpy(path2,dir_path);
    char name2[17] = "/camera_2.txt";
    strcat(path2,name2);

    FILE *pTxtFile2 = NULL;

    pTxtFile2 = fopen(path2, "w+");
    if (pTxtFile2 == NULL)
    {
        printf("The program exist!\n");
        return 0;
    }

    for(int i=0;i<5;i++)
    {
        for(int j=0;j<6;j++)
        {
            fprintf(pTxtFile2,"%d#\n",camera2_video.color_threshold[i][j]);
        }
    }

    fclose(pTxtFile2);
    */

    QMessageBox message_box(QMessageBox::Warning,"提示","保存成功!", QMessageBox::Ok, NULL);
    message_box.exec();
    return 1;

}

/*********视频显示槽***********/
void MainWindow::camera_Image_Slot()
{
    ui->label_Camera->setPixmap(QPixmap::fromImage(camera_video.image));//视频
}

void MainWindow::on_pushButton_Open_Video_clicked()
{
    if(camera_video.openCamara())
    {
        ui->pushButton_Open_Video->setEnabled(false);
        ui->pushButton_Close_Video->setEnabled(true);
    }
    else
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","未检测到1号摄像头!", QMessageBox::Ok, NULL);
        message_box.exec();
    }
    ui->pushButton_Auto_Position->setEnabled(true);
    ui->pushButton_Mannual_Position->setEnabled(true);
}


void MainWindow::on_pushButton_Close_Video_clicked()
{
    camera_video.closeCamara();
    ui->pushButton_Open_Video->setEnabled(true);
    ui->pushButton_Close_Video->setEnabled(false);
    ui->pushButton_Auto_Position->setEnabled(false);
    ui->pushButton_Mannual_Position->setEnabled(false);
}

void MainWindow::on_checkBox_clicked()
{
    if(ui->checkBox->isChecked())
    {
        on_pushButton_Range_Update_clicked();
        camera_video.bool_fill_color = true;
    }
    else camera_video.bool_fill_color = false;
}


void MainWindow::on_pushButton_Open_Video_2_clicked()
{
    if(camera2_video.openCamara())
    {
        ui->pushButton_Open_Video_2->setEnabled(false);
        ui->pushButton_Close_Video_2->setEnabled(true);
    }
    else
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","未检测到2号摄像头!", QMessageBox::Ok, NULL);
        message_box.exec();
    }
    ui->pushButton_Auto_Position_2->setEnabled(true);
    ui->pushButton_Mannual_Position_2->setEnabled(true);

}


void MainWindow::on_pushButton_Close_Video_2_clicked()
{
    camera2_video.closeCamara();
    ui->pushButton_Open_Video_2->setEnabled(true);
    ui->pushButton_Close_Video_2->setEnabled(false);
    ui->pushButton_Auto_Position_2->setEnabled(false);
    ui->pushButton_Mannual_Position_2->setEnabled(false);
}

void MainWindow::camera2_Image_Slot()
{
    ui->label_Camera_2->setPixmap(QPixmap::fromImage(camera2_video.image));//视频
}

void MainWindow::camera_calibration_Image_Slot()
{
    ui->label_Camera_Calibration->setPixmap(QPixmap::fromImage(camera_calibration.image));//视频
}

void MainWindow::on_pushButton_Open_Video_Calibration_clicked()
{
    camera_calibration.camera_seq = ui->comboBox_Camera_Calibration->currentIndex()+1;
    if(camera_calibration.openCamara())
    {
        ui->pushButton_Open_Video_Calibration->setEnabled(false);
        ui->pushButton_Close_Video_Calibration->setEnabled(true);
        ui->pushButton_Video_Calibration_Save->setEnabled(true);
        ui->pushButton_Video_Calibration_Start->setEnabled(false);
    }
    else
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","未检测到该摄像头!", QMessageBox::Ok, NULL);
        message_box.exec();
    }
}

void MainWindow::on_checkBox_2_clicked()
{
    if(ui->checkBox_2->isChecked())
    {
        on_pushButton_Range_Update_2_clicked();
        camera2_video.bool_fill_color = true;
    }
    else camera2_video.bool_fill_color = false;
}

void MainWindow::on_pushButton_Close_Video_Calibration_clicked()
{
    camera_calibration.closeCamara();
    ui->pushButton_Open_Video_Calibration->setEnabled(true);
    ui->pushButton_Close_Video_Calibration->setEnabled(false);
    ui->pushButton_Video_Calibration_Save->setEnabled(false);
    ui->pushButton_Video_Calibration_Start->setEnabled(false);
}

void MainWindow::on_comboBox_Camera_Calibration_currentIndexChanged(int index)
{
    camera_calibration.camera_seq = index + 1;
}

void MainWindow::on_pushButton_Video_Calibration_Save_clicked()
{
    camera_calibration.takingPictures();
    if(camera_calibration.total_number > 3) ui->pushButton_Video_Calibration_Start->setEnabled(true);
}

void MainWindow::calibration_Save_Image_Slot()
{
    ui->textBrowser_Calibratiob->append(tr("Get Image ")+QString::number(camera_calibration.image_counter));
}

void MainWindow::on_pushButton_Video_Calibration_Start_clicked()
{
    camera_calibration.cube_length = ui->comboBox_Cube_Length->currentIndex() + 3;
    camera_calibration.cube_width = camera_calibration.cube_length;

    camera_calibration.calibration();
}


void MainWindow::on_pushButton_Range_Update_clicked()
{
    camera_video.color_threshold[0][0] = ui->horizontalSlider_H_Min_Blue->value();
    camera_video.color_threshold[0][1] = ui->horizontalSlider_H_Max_Blue->value();
    camera_video.color_threshold[0][2] = ui->horizontalSlider_S_Min_Blue->value();
    camera_video.color_threshold[0][3] = ui->horizontalSlider_S_Max_Blue->value();
    camera_video.color_threshold[0][4] = ui->horizontalSlider_V_Min_Blue->value();
    camera_video.color_threshold[0][5] = ui->horizontalSlider_V_Max_Blue->value();

    camera_video.color_threshold[1][0] = ui->horizontalSlider_H_Min_Yellow->value();
    camera_video.color_threshold[1][1] = ui->horizontalSlider_H_Max_Yellow->value();
    camera_video.color_threshold[1][2] = ui->horizontalSlider_S_Min_Yellow->value();
    camera_video.color_threshold[1][3] = ui->horizontalSlider_S_Max_Yellow->value();
    camera_video.color_threshold[1][4] = ui->horizontalSlider_V_Min_Yellow->value();
    camera_video.color_threshold[1][5] = ui->horizontalSlider_V_Max_Yellow->value();

    camera_video.color_threshold[2][0] = ui->horizontalSlider_H_Min_Red->value();
    camera_video.color_threshold[2][1] = ui->horizontalSlider_H_Max_Red->value();
    camera_video.color_threshold[2][2] = ui->horizontalSlider_S_Min_Red->value();
    camera_video.color_threshold[2][3] = ui->horizontalSlider_S_Max_Red->value();
    camera_video.color_threshold[2][4] = ui->horizontalSlider_V_Min_Red->value();
    camera_video.color_threshold[2][5] = ui->horizontalSlider_V_Max_Red->value();

    camera_video.color_threshold[3][0] = ui->horizontalSlider_H_Min_White->value();
    camera_video.color_threshold[3][1] = ui->horizontalSlider_H_Max_White->value();
    camera_video.color_threshold[3][2] = ui->horizontalSlider_S_Min_White->value();
    camera_video.color_threshold[3][3] = ui->horizontalSlider_S_Max_White->value();
    camera_video.color_threshold[3][4] = ui->horizontalSlider_V_Min_White->value();
    camera_video.color_threshold[3][5] = ui->horizontalSlider_V_Max_White->value();

    camera_video.color_threshold[4][0] = ui->horizontalSlider_H_Min_Black->value();
    camera_video.color_threshold[4][1] = ui->horizontalSlider_H_Max_Black->value();
    camera_video.color_threshold[4][2] = ui->horizontalSlider_S_Min_Black->value();
    camera_video.color_threshold[4][3] = ui->horizontalSlider_S_Max_Black->value();
    camera_video.color_threshold[4][4] = ui->horizontalSlider_V_Min_Black->value();
    camera_video.color_threshold[4][5] = ui->horizontalSlider_V_Max_Black->value();

}

void MainWindow::on_pushButton_Range_Update_2_clicked()
{
    camera2_video.color_threshold[0][0] = ui->horizontalSlider_H_Min_Blue_2->value();
    camera2_video.color_threshold[0][1] = ui->horizontalSlider_H_Max_Blue_2->value();
    camera2_video.color_threshold[0][2] = ui->horizontalSlider_S_Min_Blue_2->value();
    camera2_video.color_threshold[0][3] = ui->horizontalSlider_S_Max_Blue_2->value();
    camera2_video.color_threshold[0][4] = ui->horizontalSlider_V_Min_Blue_2->value();
    camera2_video.color_threshold[0][5] = ui->horizontalSlider_V_Max_Blue_2->value();

    camera2_video.color_threshold[1][0] = ui->horizontalSlider_H_Min_Yellow_2->value();
    camera2_video.color_threshold[1][1] = ui->horizontalSlider_H_Max_Yellow_2->value();
    camera2_video.color_threshold[1][2] = ui->horizontalSlider_S_Min_Yellow_2->value();
    camera2_video.color_threshold[1][3] = ui->horizontalSlider_S_Max_Yellow_2->value();
    camera2_video.color_threshold[1][4] = ui->horizontalSlider_V_Min_Yellow_2->value();
    camera2_video.color_threshold[1][5] = ui->horizontalSlider_V_Max_Yellow_2->value();

    camera2_video.color_threshold[2][0] = ui->horizontalSlider_H_Min_Red_2->value();
    camera2_video.color_threshold[2][1] = ui->horizontalSlider_H_Max_Red_2->value();
    camera2_video.color_threshold[2][2] = ui->horizontalSlider_S_Min_Red_2->value();
    camera2_video.color_threshold[2][3] = ui->horizontalSlider_S_Max_Red_2->value();
    camera2_video.color_threshold[2][4] = ui->horizontalSlider_V_Min_Red_2->value();
    camera2_video.color_threshold[2][5] = ui->horizontalSlider_V_Max_Red_2->value();

    camera2_video.color_threshold[3][0] = ui->horizontalSlider_H_Min_White_2->value();
    camera2_video.color_threshold[3][1] = ui->horizontalSlider_H_Max_White_2->value();
    camera2_video.color_threshold[3][2] = ui->horizontalSlider_S_Min_White_2->value();
    camera2_video.color_threshold[3][3] = ui->horizontalSlider_S_Max_White_2->value();
    camera2_video.color_threshold[3][4] = ui->horizontalSlider_V_Min_White_2->value();
    camera2_video.color_threshold[3][5] = ui->horizontalSlider_V_Max_White_2->value();

    camera2_video.color_threshold[4][0] = ui->horizontalSlider_H_Min_Black_2->value();
    camera2_video.color_threshold[4][1] = ui->horizontalSlider_H_Max_Black_2->value();
    camera2_video.color_threshold[4][2] = ui->horizontalSlider_S_Min_Black_2->value();
    camera2_video.color_threshold[4][3] = ui->horizontalSlider_S_Max_Black_2->value();
    camera2_video.color_threshold[4][4] = ui->horizontalSlider_V_Min_Black_2->value();
    camera2_video.color_threshold[4][5] = ui->horizontalSlider_V_Max_Black_2->value();
}

void MainWindow::on_radioButton_left_1_clicked()
{
    camera_video.camera_left_side = true;
}

void MainWindow::on_radioButton_right_1_clicked()
{
    camera_video.camera_left_side = false;
}


void MainWindow::on_radioButton_left_2_clicked()
{
    camera2_video.camera_left_side = true;
}

void MainWindow::on_radioButton_right_2_clicked()
{
    camera2_video.camera_left_side = false;
}

void MainWindow::on_horizontalSlider_Height_Threshold_sliderMoved(int position)
{
    camera_video.height_threshold = ui->horizontalSlider_Height_Threshold->value();
}

void MainWindow::on_horizontalSlider_Height_Threshold_2_sliderMoved(int position)
{
    camera2_video.height_threshold = ui->horizontalSlider_Height_Threshold_2->value();
}

void MainWindow::on_pushButton_Auto_Position_clicked()
{
    camera_video.capture = true;

    QMessageBox::StandardButton button;
    button = QMessageBox::question(this, tr("注意"),
                                   QString(tr("满足要求?")),
                                   QMessageBox::Yes | QMessageBox::No);
    if (button == QMessageBox::No) {
        camera_video.bool_cut = false;
    }
    else if (button == QMessageBox::Yes) {
        camera_video.bool_cut = true;
    }
}

void MainWindow::on_pushButton_Mannual_Position_clicked()
{
    camera_video.mannual_position();
}

void MainWindow::on_radioButton_enemy_1_clicked()
{
    camera_video.camera_enemy_side  = true;
}

void MainWindow::on_radioButton_ours_1_clicked()
{
    camera_video.camera_enemy_side  = false;
}

void MainWindow::on_radioButton_enemy_2_clicked()
{
    camera2_video.camera_enemy_side = true;
}

void MainWindow::on_radioButton_ours_2_clicked()
{
    camera2_video.camera_enemy_side = false;
}



void MainWindow::on_pushButton_Auto_Position_2_clicked()
{
    camera2_video.capture = true;

    QMessageBox::StandardButton button;
    button = QMessageBox::question(this, tr("注意"),
                                   QString(tr("满足要求?")),
                                   QMessageBox::Yes | QMessageBox::No);
    if (button == QMessageBox::No) {
        camera2_video.bool_cut = false;
    }
    else if (button == QMessageBox::Yes) {
        camera2_video.bool_cut = true;
    }
}

void MainWindow::on_pushButton_Mannual_Position_2_clicked()
{
    camera2_video.mannual_position();
}

void MainWindow::on_checkBox_cut_area_clicked(bool checked)
{
    if(checked) camera_video.bool_cut_ticked = true;
    else camera_video.bool_cut_ticked = false;
}



void MainWindow::on_checkBox_cut_area_2_clicked(bool checked)
{
    if(checked) camera2_video.bool_cut_ticked = true;
    else camera2_video.bool_cut_ticked = false;
}
