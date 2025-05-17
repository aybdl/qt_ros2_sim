/**
 * @file /include/qt_ros_test/main_window.hpp
 *
 * @brief Qt based gui for qt_ros_test.
 *
 * @date November 2010
 **/
#ifndef qt_ros_test_MAIN_WINDOW_H
#define qt_ros_test_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "qrviz.hpp"
#include "my_GraphicsView.hpp"
#include <QtGui>
#include "CConnectionForSshClient.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include "CCtrlDashBoard.hpp"
#include <QTimer>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_ros_test {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    qrviz* myrviz;

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void init_cmd();
    void init_ssh();
    void btn_disableall(bool _enabled);
    void closeroslaunch();
Q_SIGNALS:
    void sigSend(QString strMsg);

public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
    void slot_cmd_read();
    void slot_cmd_err();

    void slot_rosOpen();

    void cmd_finished(int exitCode, QProcess::ExitStatus exitStatus);

private slots:


    //void on_pushButton_clicked();

    void on_checkBox_key_clicked(bool checked);

    void pushButton_go();

    //void pushButton_stop();

    void updata_image(QImage);

    void slotbatteryState(double);


    void slotspeedState(double,double,double,double,double,double);

    void slotDataArrived(QString strMsg,QString strIp, int nPort);

    void on_horizontalSlider_v_sliderMoved(int position);

    void on_horizontalSlider_c_sliderMoved(int position);

    void on_checkBox__opencamera_clicked(bool checked);

    void on_pushButton_base_clicked(bool checked);

    void on_pushButton_laser_clicked(bool checked);

    void on_pushButton_joy_clicked(bool checked);

    void on_pushButton_linefollower_clicked(bool checked);

    void on_pushButton_laserfollower_clicked(bool checked);

    void on_pushButton_clicked(bool checked);

    void on_pushButton_subimg_clicked(bool checked);

    void on_pushButton_visfollower_clicked(bool checked);

    void on_pushButton_webusbcam_clicked(bool checked);

    void on_pushButton_2dmap_clicked(bool checked);

    void on_pushButton_2dmapsave_clicked(bool checked);

    void on_pushButton_2dnav_clicked(bool checked);

    void on_pushButton_3dmap_clicked(bool checked);

    void on_pushButton_3dnav_clicked(bool checked);

    void on_pushButton_2_clicked(bool checked);

    void on_lineEdit_cmd_returnPressed();

    void on_pushButton_nfsrobot_clicked(bool checked);

    void on_pushButton_unfsrobot_clicked(bool checked);

    void on_pushButton_nfsarm_clicked(bool checked);

    void on_pushButton_unfsarm_clicked(bool checked);

    void on_pushButton_settime_clicked(bool checked);

    void on_pushButton_catkinmake_clicked(bool checked);

    void on_pushButton_catkinmakepkg_clicked(bool checked);

    void on_pushButton_rqt_clicked(bool checked);

    void on_pushButton_rviz_clicked(bool checked);

    void keyPressEvent(QKeyEvent *event);

    void keyReleaseEvent(QKeyEvent *event);

    void slottimeout();

    void on_quit_button_2_clicked(bool checked);

    void on_pushButton_2dmap_2_clicked();

    void on_pushButton_2dmap_3_clicked();

    void slotConnectStateChanged(bool bState,QString strIp,int nPort);

    void on_pushButton_connect_che_clicked();



private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    MyGraphicsView* mygraphicsView;
    QProcess *_cmd = NULL;
    QProcess *killall = NULL;
    bool m_bConnectState;
    CConnectionForSshClient *m_sshSocket;
    CCtrlDashBoard *speedxboard;
    CCtrlDashBoard *speedyboard;
    QTimer* m_pTimer;
    std::string user_passward;
    bool keypress;


    //2025.5.15后新加
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
    geometry_msgs::msg::Twist currentTwist_;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub;
    QTimer *battery_timer;
    double current_voltage = 0.0;
    bool low_battery_warning_shown = false;

    void velControl();
    double speed;//线速度
    double cor;//角速度
};

}  // namespace qt_ros_test

#endif // qt_ros_test_MAIN_WINDOW_H
