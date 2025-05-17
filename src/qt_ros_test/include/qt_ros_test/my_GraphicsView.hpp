#ifndef MY_GRAPHICSVIEW_HPP
#define MY_GRAPHICSVIEW_HPP

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QImage>
#include <QPixmap>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

class MyGraphicsView : public QGraphicsView {
    Q_OBJECT

public:
    explicit MyGraphicsView(QWidget *parent = nullptr);
    ~MyGraphicsView() override;
public slots:
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void updatePosition(const geometry_msgs::msg::Pose::SharedPtr pose);
    void updateTrajectory(const geometry_msgs::msg::Pose::SharedPtr pose);

private:
    QGraphicsScene* scene;
    QGraphicsPixmapItem* mapItem = nullptr;
    QGraphicsPathItem* trajectoryItem = nullptr;
    QGraphicsEllipseItem* positionItem = nullptr;
    QVector<QPointF> trajectory; // 存储轨迹
    double origin_x=-10005;
    double origin_y;
    double my_resolution;
    double map_height;
};

#endif // MY_GRAPHICSVIEW_HPP

