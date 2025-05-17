/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include"qnode.hpp"
#include "../include/qt_ros_test/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    qt_ros_test::MainWindow mainwindow(argc,argv);

    // qrviz* Myrviz=mainwindow.myrviz;
    // QNode node(Myrviz);

    mainwindow.setFixedSize(1440,960);
    mainwindow.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    rclcpp::shutdown();

	return result;
}
