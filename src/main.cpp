#include<stdio.h>
#include<qapplication.h>

//#include<QtWidgets>
#include "viewer.h"
// ROS includes 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char *argv[]){
    QApplication application(argc, argv);

    Viewer viewer;

    viewer.setWindowTitle("viewer");

    viewer.show();

    return application.exec();
}
/**
int main(int argc, char* argv[]){
    QApplication app(argc, argv);

    QTextEdit textEdit;
    QPushButton quitButton("Quit");

    QObject::connect(&quitButton, SIGNAL (clicked()), qApp, SLOT (quit()));

    QVBoxLayout layout;
    layout.addWidget(&textEdit);
    layout.addWidget(&quitButton);

    QWidget window;
    window.setLayout(&layout);

    window.show();

    return app.exec();
}
**/
