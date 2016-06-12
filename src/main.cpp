//*********main.cpp**********

#include "mainwindow.h"
#include <QApplication>
#include "ros/ros.h"
#include "receiver.h"
#include "painterWiget.h"
#include "camera.h"
#include "camera2.h"
#include <QTextCodec>

MavrosMessage message;
Camera camera_video;
Camera camera2_video;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //中文字符设置
    QTextCodec *codec = QTextCodec::codecForName("UTF-8");
    QTextCodec::setCodecForTr(codec);
    QTextCodec::setCodecForLocale(QTextCodec::codecForLocale());
    QTextCodec::setCodecForCStrings(QTextCodec::codecForLocale());

    MainWindow w;
    w.showMaximized();//最大化显示

    ros::init(argc,argv,"receiver");
    message.start();

    return a.exec();
}
