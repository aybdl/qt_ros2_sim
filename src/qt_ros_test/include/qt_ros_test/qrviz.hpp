#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <QWidget>
#include <OgreCamera.h>
#include <QThread>
#include <QApplication>
#include "rclcpp/rclcpp.hpp"
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/view_manager.hpp>
#include "rviz_common/properties/quaternion_property.hpp"
#include"dummywindowmanager.hpp"
#include "rviz_common/display_group.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>





#include "rviz_rendering/render_window.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include"rviz_common/properties/float_property.hpp"

#include <QVBoxLayout>
class qrviz : public QWidget//QThread // ← 一定要继承 QWidget
{
    Q_OBJECT
public:
    //qrviz();
    qrviz(QWidget * parent = nullptr);
    std::shared_ptr<rviz_common::RenderPanel> _render_panel;
    std::shared_ptr<rviz_common::VisualizationManager> _manager;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Cmd_Vel_Sub;


    void updateMapOnMainThread(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void batteryCallback(const std_msgs::msg::Float32::SharedPtr message);
    void speedCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void pub_cmd(bool car_model,int keybutton,double speed,double turn);
    void sub_camera(QString cameratopic);
    void image1_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void image2_callback(const sensor_msgs::msg::CompressedImage &msg);

    QImage Mat2QImage(cv::Mat const& src);


    void setupTFDisplay(bool tf);


signals:
    // 新增的信号声明
    void mapDataReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void batteryStateReceived(double);
    void speedState(double,double,double,double,double,double);
    void image_val(QImage);


// public slots:
//     void UpdateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
private slots:
    void updateMapReceivedIndicator(bool received);

protected:
    void run();
private:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> _rvizRosNodeTmp;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr _rvizRosNode;
    rviz_common::Display* _occupancy_grid_display;

    //图像订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;//非压缩图像
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imagecomp_sub;//压缩图像

    rviz_common::Display * grid_;
    rviz_common::Display * _pointcloud;
    rviz_common::Display *tf_display_;       // TF display object
    rviz_common::Display *map_display_;      // Map display object
    rviz_common::Display *robot_model_display_; // RobotModel display object

    std::shared_ptr<DummyWindowManager> _windowManager;


    geometry_msgs::msg::Twist currentTwist_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscriber_; // Subscriber for map data
    bool mapReceived_;                       // Boolean flag to track map data reception

    QString subcameratopic;


    void setupGridDisplay();

    void setupMapDisplay();
    void setupRobotModelDisplay();
    void setupMapSubscriber();
    void setupLaserScanDisplay();



};

#endif // QRVIZ_HPP
