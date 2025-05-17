#include "../include/qt_ros_test/my_GraphicsView.hpp"
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QPainter>
#include <QDebug>
#include <cmath>

MyGraphicsView::MyGraphicsView(QWidget *parent)
    : QGraphicsView(parent),
    scene(new QGraphicsScene(this)) {
    setScene(scene);
    setRenderHint(QPainter::Antialiasing);
    setRenderHint(QPainter::SmoothPixmapTransform);
}

MyGraphicsView::~MyGraphicsView() {
    delete scene;
}

void MyGraphicsView::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    QImage img(map->info.width, map->info.height, QImage::Format_RGB888);
    for (unsigned int y = 0; y < map->info.height; ++y) {
        for (unsigned int x = 0; x < map->info.width; ++x) {
            int index = y * map->info.width + x;
            int color;

            if (map->data[index] == 0)
                color = 255; // Free
            else if (map->data[index] == 100)
                color = 0;   // Occupied
            else
                color = 127; // Unknown

            int flipped_y = map->info.height - 1 - y; // 翻转Y坐标
            img.setPixel(x, flipped_y, qRgb(color, color, color));
        }
    }

    if (mapItem) {
        scene->removeItem(mapItem);
        delete mapItem;
    }

    mapItem = scene->addPixmap(QPixmap::fromImage(img));
    mapItem->setScale(4.0);
    mapItem->setZValue(0); // 确保地图在底层
    origin_x = map->info.origin.position.x;
    origin_y = map->info.origin.position.y;
    my_resolution=map->info.resolution;
    map_height=map->info.height;
}

void MyGraphicsView::updatePosition(const geometry_msgs::msg::Pose::SharedPtr pose) {
    // 绘制当前位置


    // 打印出来

    //qDebug() << "Received Pose:" << "x =" << x << ", y =" << y << ", z =" << z;
    double scale = 4.0;

    // 计算地图坐标位置
    double px = (pose->position.x - origin_x) * scale/my_resolution;
    //double py = (pose->position.y - origin_y) * scale/my_resolution;
    double py = (map_height - (pose->position.y - origin_y) / my_resolution) * scale;

    //qDebug() << "Received Pose:" << "x =" << px << ", y =" << py;

    // 计算位置时确保不偏移，视图的原点通常在左上角，所以要处理偏移
    if (positionItem) {
        scene->removeItem(positionItem);
        delete positionItem;
        positionItem = nullptr;
    }

    // 添加新位置图元
    // positionItem = new QGraphicsEllipseItem(0, 0, 10, 10);
    // positionItem->setBrush(Qt::red);
    // positionItem->setPen(QPen(Qt::black));
    // positionItem->setZValue(2.0); // 高于轨迹和地图
    // positionItem->setPos(px, py);
    // scene->addItem(positionItem);


    positionItem = new QGraphicsEllipseItem(px-5, py-5, 10, 10); // 半径5，直径10
    positionItem->setBrush(Qt::red);
    positionItem->setPen(QPen(Qt::black)); // 黑色描边，更明显
    positionItem->setZValue(1.0); // 提升到地图上层
    scene->addItem(positionItem);
}

void MyGraphicsView::updateTrajectory(const geometry_msgs::msg::Pose::SharedPtr pose) {
    // 绘制轨迹
    if (!pose || !scene || my_resolution <= 0.0||origin_x<-10000) {
        qDebug() << "[Trajectory] Not ready: pose/scene/map/resolution invalid";
        return;
    }
    double scale = 4.0;

    // 世界坐标 → 地图像素坐标
    double px = (pose->position.x - origin_x) * scale / my_resolution;
    double py = (map_height - (pose->position.y - origin_y) / my_resolution) * scale;


    if (!std::isfinite(px) || !std::isfinite(py)) {
        qDebug() << "[Trajectory] Invalid coordinates! px:" << px << ", py:" << py;
        return;
    }
    QPointF newPoint(px, py);

    // 添加新轨迹点，避免抖动
    if (trajectory.isEmpty() || QLineF(trajectory.last(), newPoint).length() > 0.5) {
        trajectory.append(newPoint);
    }

    if (trajectory.isEmpty()) return;

    // 构建轨迹路径
    QPainterPath path;
    path.moveTo(trajectory.first());
    for (const QPointF& point : trajectory) {
        if (std::isfinite(point.x()) && std::isfinite(point.y()))
            path.lineTo(point);
    }

    // 初始化轨迹图元
    if (!trajectoryItem) {
        trajectoryItem = new QGraphicsPathItem();

        QColor lineColor = Qt::blue;  // 🔵 可自定义颜色
        trajectoryItem->setPen(QPen(lineColor, 1.0));
        trajectoryItem->setZValue(1.5);

        scene->addItem(trajectoryItem);
        qDebug() << "[Trajectory] trajectoryItem added to scene.";
    }

    // 更新路径
    trajectoryItem->setPath(path);
}
