#include "../include/qt_ros_test/qrviz.hpp"
#include <QDebug>
#include <QException>
#include <OgreCamera.h>
#include<QMouseEvent>
#include <OgreQuaternion.h>
#include <QVector3D>


qrviz::qrviz(QWidget * parent)
{
    mapReceived_=false;

    _rvizRosNodeTmp = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
    _rvizRosNode = _rvizRosNodeTmp;

    QApplication::processEvents();
    _render_panel = std::make_shared<rviz_common::RenderPanel>();
    QApplication::processEvents();
    _render_panel->getRenderWindow()->initialize();

    _windowManager = std::make_shared<DummyWindowManager>(parent);
    // /获取 WindowManagerInterface 指针
    rviz_common::WindowManagerInterface* wm = _windowManager.get();

    auto clock = _rvizRosNode.lock()->get_raw_node()->get_clock();


    _manager = std::make_shared<rviz_common::VisualizationManager>(_render_panel.get(), _rvizRosNode, wm, clock);
    _render_panel->initialize(_manager.get());
    QApplication::processEvents();

    _manager->setFixedFrame("map");
    _manager->initialize();
    _manager->startUpdate();



    //初始化发布者和订阅者
    cmdVelPublisher_ = _rvizRosNode.lock()->get_raw_node()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    battery_sub = _rvizRosNode.lock()->get_raw_node()->create_subscription<std_msgs::msg::Float32>("/PowerVoltage", 5,std::bind(&qrviz::batteryCallback, this, std::placeholders::_1));
    Cmd_Vel_Sub = _rvizRosNode.lock()->get_raw_node()->create_subscription<nav_msgs::msg::Odometry>("/odom", 5,std::bind(&qrviz::speedCallback, this, std::placeholders::_1));

    sub_camera("/camera/color/image_raw/compressed");

    //初始化TF树，默认不显示
    tf_display_ = _manager->createDisplay("rviz_default_plugins/TF", "TF Display", false);
    // Set up the TF display to show frames with a fixed frame
    if (tf_display_) {
        tf_display_->subProp("Show Axes")->setValue(true);
        tf_display_->subProp("Show Names")->setValue(true); // 显示坐标系名称
        qDebug() << "TF display configured with axes and names shown.";
    } else {
        qDebug() << "Failed to create TF display.";
    }


    // Set the view controller to Orbit to allow for mouse interactions
    _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");

    // Retrieve the active view controller to set properties and confirm it's set up correctly
    auto orbit_view_controller = _manager->getViewManager()->getCurrent();
    if (!orbit_view_controller) {
        qDebug() << "Orbit view controller could not be set.";
        return;
    }

    qDebug() << "Orbit view controller initialized successfully.";

    // Set default distance and focal point for the camera
    orbit_view_controller->subProp("Distance")->setValue(10.0);
    orbit_view_controller->subProp("Focal Point")->setValue(QVariant::fromValue(QVector3D(0.0, 0.0, 0.0)));

    // Set initial orientation of the camera
    orbit_view_controller->subProp("Pitch")->setValue(1.5708);  // Example angle in radians
    orbit_view_controller->subProp("Yaw")->setValue(3.14);     // Example angle in radians

    // Set Interact tool as the active tool to enable mouse interactions
    auto tool_manager = _manager->getToolManager();


    tool_manager->setCurrentTool(tool_manager->addTool("rviz_default_plugins/Interact"));

    // 添加网格显示
    setupGridDisplay();

    // 添加TF显示（坐标系关系）
    //setupTFDisplay();

    setupMapDisplay();
    setupRobotModelDisplay();
    //setupLaserScanDisplay();
    setupMapSubscriber();
}

void qrviz::setupGridDisplay()
{
    //QString frame_id = frameLineEdit_->text();

    // Initialize the grid display
    grid_ = _manager->createDisplay("rviz_default_plugins/Grid", "Grid", true);
    if (grid_) {
        grid_->subProp("Line Style")->setValue("Lines");
        grid_->subProp("Color")->setValue(QColor(Qt::white));
        grid_->subProp("Cell Size")->setValue(0.5f);
        //qDebug() << "Grid display configured for fixed frame:" ;
    } else {
        qDebug() << "Failed to create Grid display.";
    }
}

void qrviz::setupTFDisplay(bool visible)
{

    //设置TF显示的可见性
    tf_display_->setEnabled(visible);

    // 打印状态信息
    qDebug() << "TF display visibility set to:" << visible;
}
// void qrviz::setupTFDisplay(bool visible)
// {
//     // 第一次调用时创建TF显示插件
//     if (!tf_display_) {
//         tf_display_ = _manager->createDisplay("rviz_default_plugins/TF", "TF Display", true);
//         if (tf_display_) {
//             tf_display_->subProp("Show Axes")->setValue(true);
//             tf_display_->subProp("Show Names")->setValue(true); // 显示坐标系名称
//             qDebug() << "TF display created and configured.";
//         } else {
//             qDebug() << "Failed to create TF display.";
//             return;
//         }
//     }

//     // 设置TF显示的可见性
//     tf_display_->setEnabled(visible);

//     // 打印状态信息
//     qDebug() << "TF display visibility set to:" << visible;
// }

void qrviz::setupMapDisplay()
{
    // Set up the Map display for the /map topic
    map_display_ = _manager->createDisplay("rviz_default_plugins/Map", "Map Display", true);
    if (map_display_) {
        map_display_->setProperty("Texture Format", "RGBA8");
        map_display_->subProp("Topic")->setValue("/map");
        map_display_->subProp("Alpha")->setValue(1.0);
        map_display_->subProp("Draw Behind")->setValue(false);
        map_display_->subProp("Color Scheme")->setValue("map");
        map_display_->subProp("Topic")->subProp("Durability Policy")->setValue("Transient Local");



        //map_display_->setEnabled(true);

        qDebug() << "Map display configured for /map topic ";
    } else {
        qDebug() << "Failed to create Map display.";
    }
}

void qrviz::setupRobotModelDisplay()
{
    // Set up the RobotModel display for the /robot_description topic
    robot_model_display_ = _manager->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", true);
    if (robot_model_display_) {
        // robot_model_display_->subProp("Description Topic")->setValue("/tb3_0/robot_description");  // Set the topic to /robot_description
        // robot_model_display_->subProp("TF Prefix")->setValue("");  // Set TF prefix to empty if needed /tb3_0/robot_description
        // qDebug() << "RobotModel display configured for /robot_description topic.";


        // 设置为使用话题方式
        //robot_model_display_->subProp("Description Source")->setValue("/robot_description");

        // 设置为你的机器人发布的话题（根据你的描述，使用/robot_description）
        robot_model_display_->subProp("Description Topic")->setValue("/robot_description");

        // 设置对应的TF前缀（如果你的机器人有统一的前缀，否则留空）
        robot_model_display_->subProp("TF Prefix")->setValue("");

        qDebug() << "RobotModel display configured to subscribe to /robot_description topic.";
    } else {
        qDebug() << "Failed to create RobotModel display.";
    }
}


void qrviz::setupMapSubscriber()
{
    auto node = _rvizRosNode.lock()->get_raw_node();
    mapSubscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            Q_UNUSED(msg);
            mapReceived_ = true;
            qDebug() << "Map Received";
            updateMapReceivedIndicator(true);

            // Enable map display if map data is received
            /*if (map_display_) {
                map_display_->setEnabled(true);
            }*/
        }
        );
}


void qrviz::updateMapReceivedIndicator(bool received) {
    if (received) {
        // mapReceivedIndicator_->setText("Map Received: Yes");
        // mapReceivedIndicator_->setStyleSheet("color: green;");
    } else {
        // mapReceivedIndicator_->setText("Map Received: No");
        // mapReceivedIndicator_->setStyleSheet("color: red;");
    }
}

void qrviz::setupLaserScanDisplay()
{
    auto laser_scan_display = _manager->createDisplay("rviz_default_plugins/LaserScan", "LaserScan Display", true);
    if (laser_scan_display) {
        laser_scan_display->subProp("Topic")->setValue("/scan");       // Set to the topic where laser data is published
        laser_scan_display->subProp("Size (m)")->setValue(0.1);        // Adjust point size as needed
        laser_scan_display->subProp("Color")->setValue(QColor(Qt::green));  // Set color of laser points
        qDebug() << "LaserScan display configured successfully for /scan.";
    } else {
        qDebug() << "Failed to configure LaserScan display.";
    }
}


void qrviz::run()
{
    // rclcpp::WallRate loop_rate(10); // 10HZ
    while (rclcpp::ok())
    {
        QApplication::processEvents();
        // rclcpp::spin_some(_rvizRosNode.lock()->get_raw_node());
        // loop_rate.sleep();
    }
    // rclcpp::shutdown();
}

void qrviz::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    emit batteryStateReceived(msg->data);
}

void qrviz::speedCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    emit speedState(odom->twist.twist.linear.x,odom->twist.twist.linear.y,odom->twist.twist.linear.z,odom->twist.twist.angular.x,odom->twist.twist.angular.y,odom->twist.twist.angular.z);
}



/*
 * 函数：按钮控制小车运动
 * car_model:0:麦轮小车  1：全向轮
 * keybutton：按钮返回值    u i o
 *                        j k l
 *                        m , .
 * speed:线速度
 * turn:角速度
*/
void qrviz::pub_cmd(bool car_model, int keybutton, double speed, double turn)
{
    float x,th;
    switch (keybutton)
    {
    case 2:x=1;th=0;break;
    case 3:x=1;th=-1;break;
    case 4:x=0;th=1;break;
    case 6:x=0;th=-1;break;
    case 1:x=1;th=1;break;
    case 8:x=-1;th=0;break;
    case 9:
        if(car_model==0){x=-1;th=-1;}
        else if(car_model==1) {x=-1;th=1;}
        break;
    case 7:
        if(car_model==0){x=-1;th=1;}
        else if(car_model==1) {x=-1;th=-1;}
        break;
    case 5:x=0;th=0;break;
    default:x=0;th=0;break;
    }
    double target_speed = speed * x;
    double target_turn  = turn * th;
    double target_HorizonMove = speed*th;

    geometry_msgs::msg::Twist twist;
    if(car_model==0)
    {
        twist.linear.x  = target_speed;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = target_turn;
    }
    else
    {
        twist.linear.x  = target_speed;
        twist.linear.y = target_HorizonMove;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
    qDebug()<<car_model<<keybutton<<speed<<turn<<target_speed<<target_turn<<target_HorizonMove;
    cmdVelPublisher_->publish(twist);
}


void qrviz::sub_camera(QString cameratopic)
{
    subcameratopic = cameratopic;
    if(cameratopic.contains("compress"))
    {
        static auto imagecomp_sub = _rvizRosNode.lock()->get_raw_node()->create_subscription<sensor_msgs::msg::CompressedImage>(cameratopic.toStdString(),5,std::bind(&qrviz::image2_callback, this, std::placeholders::_1));
        imagecomp_sub= nullptr;
        imagecomp_sub = _rvizRosNode.lock()->get_raw_node()->create_subscription<sensor_msgs::msg::CompressedImage>(cameratopic.toStdString(),5,std::bind(&qrviz::image2_callback, this, std::placeholders::_1));
    }
    else
    {
        static auto image_sub =_rvizRosNode.lock()->get_raw_node()->create_subscription<sensor_msgs::msg::Image>(cameratopic.toStdString(),5,std::bind(&qrviz::image1_callback, this, std::placeholders::_1));
        image_sub= nullptr;
        image_sub = _rvizRosNode.lock()->get_raw_node()->create_subscription<sensor_msgs::msg::Image>(cameratopic.toStdString(),5,std::bind(&qrviz::image1_callback, this, std::placeholders::_1));
    }
}


void qrviz::image1_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if(subcameratopic.contains("compress"))return;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if(subcameratopic.contains("depth"))
        {
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
        }
        else  cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception & e)
    {
        qDebug()<<"image subscription error!!!"<<subcameratopic;
        return;
    }
    QImage im=Mat2QImage(cv_ptr->image);
    emit image_val(im);
}

void qrviz::image2_callback(const sensor_msgs::msg::CompressedImage &msg)
{
    if(!subcameratopic.contains("compress"))return;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if(subcameratopic.contains("depth"))
        {
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
        }
        else  cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception & e)
    {
        qDebug()<<"image compress subscription error!!!"<<subcameratopic;
        return;
    }
    QImage im=Mat2QImage(cv_ptr->image);
    emit image_val(im);
}

QImage qrviz::Mat2QImage(cv::Mat const& src)
{
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    const float scale = 255.0;
    if (src.depth() == CV_8U) {
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    int level = src.at<quint8>(i, j);
                    dest.setPixel(j, i, qRgb(level, level, level));
                }
            }
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
            }
        }
    } else if(src.depth() == CV_16U){
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    int level = (int)src.at<ushort>(i, j);
                    dest.setPixel(j, i, qRgb(level, level, level));
                }
            }
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3b bgr = src.at<cv::Vec3b>(i, j)*65535;
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
            }
        }
    }else if (src.depth() == CV_32F) {
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    int level = scale * src.at<float>(i, j);
                    dest.setPixel(j, i, qRgb(level, level, level));
                }
            }
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
            }
        }
    }

    return dest;
}
