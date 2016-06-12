//********mianwindow.cpp*********

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "receiver.h"
#include "QDesktopWidget"
#include "camera.h"
#include "camera2.h"

extern MavrosMessage message;
extern Camera camera_video;
extern Camera2 camera2_video;

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
}

int MainWindow::read_saved_paras()
{
    /*char dir_path[80]="/home/cc/catkin_ws/src/break_point";
    QDir *temp = new QDir;
    bool exist = temp->exists(QString(dir_path));
    if(!exist)
    {
        cout<<"no config file found!"<<endl;
        return 0;
    }

    QString fileName = "/home/cc/catkin_ws/src/break_point/config.txt";
    fstream config_f;
    char *path = fileName.toLatin1().data();
    config_f.open(path,ios::in);

    int read_counter=0;
    while(!config_f.eof())
    {   //while not the end of file
        char str[30];
        config_f >> str;

        float fnum = str[1]-'0'+ (str[3]-'0')/10.0;
        switch(read_counter)
        {
        case 0:
            take_off_height = fnum;
            break;
        case 1:
            spray_length = fnum;
            break;
        case 2:
            spray_width = fnum;
            break;
        default:
            break;
        }
        read_counter ++;
    }
        config_f.close(); //reading finished
        */
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
    /*char name[17] = "/config.txt";
    char path[80]="/home/cc/catkin_ws/src/break_point";

    QDir *temp = new QDir;
    bool exist = temp->exists(QString(path));
    if(!exist)temp->mkdir(QString(path));

    strcat(path,name);

    cout<<"file saved in "<<path<<endl;
    FILE *pTxtFile = NULL;

    pTxtFile = fopen(path, "w+");
    if (pTxtFile == NULL)
    {
        printf("The program exist!\n");
        return 0;
    }

    cout<<"saving...\n";

    fprintf(pTxtFile,"#%.1f#\n",take_off_height);
    fprintf(pTxtFile,"#%.1f#\n",spray_length);
    fprintf(pTxtFile,"#%.1f#\n",spray_width);

    fprintf(pTxtFile,"end");
    //fprintf(pTxtFile,"take_off_height spray_length spray_width message.extra_function.laser_height_enable message.extra_function.obs_avoid_enable message.pump.pump_speed_sp");

    fclose(pTxtFile);

    QMessageBox message_box(QMessageBox::Warning,"提示","保存成功!", QMessageBox::Ok, NULL);
    message_box.exec();
    return 1;*/

}

/*********视频显示槽***********/
void MainWindow::camera_Image_Slot()
{
    ui->label_Camera->setPixmap(QPixmap::fromImage(camera_video.image));//视频
}

void MainWindow::on_pushButton_Open_Video_clicked()
{
    camera_video.openCamara();
    ui->pushButton_Open_Video->setEnabled(false);
    ui->pushButton_Close_Video->setEnabled(true);
}


void MainWindow::on_pushButton_Close_Video_clicked()
{
    camera_video.closeCamara();
    ui->pushButton_Open_Video->setEnabled(true);
    ui->pushButton_Close_Video->setEnabled(false);
}

void MainWindow::camera2_Image_Slot()
{
    ui->label_Camera_2->setPixmap(QPixmap::fromImage(camera2_video.image));//视频
}

void MainWindow::on_pushButton_Open_Video_2_clicked()
{
    camera2_video.openCamara();
    ui->pushButton_Open_Video_2->setEnabled(false);
    ui->pushButton_Close_Video_2->setEnabled(true);
}


void MainWindow::on_pushButton_Close_Video_2_clicked()
{
    camera2_video.closeCamara();
    ui->pushButton_Open_Video_2->setEnabled(true);
    ui->pushButton_Close_Video_2->setEnabled(false);
}
