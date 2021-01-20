#include <ros/ros.h>
#include <rosThread.h>
#include <QApplication>
#include <QThread>

int main(int argc,char** argv){
    QApplication app(argc,argv);

    ros::init(argc,argv,"test_project");

    RosThread* rosthread  = new RosThread;
    QThread* worker = new QThread(&app);
    rosthread->moveToThread(worker);

    QObject::connect(worker,SIGNAL(finished()),&app,SLOT(quit()));
    QObject::connect(worker,SIGNAL(finished()),rosthread,SLOT(deleteLater()));
    QObject::connect(worker,SIGNAL(started()),rosthread,SLOT(work()));

    worker->start();

    return app.exec();
}
