#include "../include/qt_ros_test/dummywindowmanager.hpp"
#include "dummywindowmanager.hpp"
#include <rviz_common/panel_dock_widget.hpp>
#include <iostream>

/**
 * @brief 构造函数，初始化父窗口指针
 * @param parent 父窗口指针
 */
DummyWindowManager::DummyWindowManager(QWidget * parent) : parent_widget_(parent)
{
}

/**
 * @brief 析构函数
 */
DummyWindowManager::~DummyWindowManager()
{
}

/**
 * @brief 获取父窗口指针
 * @return 父窗口指针
 */
QWidget * DummyWindowManager::getParentWindow()
{
    return parent_widget_;
}

/**
 * @brief 添加一个面板到可视化界面
 * @param name 面板的名称
 * @param pane 面板的 QWidget 指针
 * @param area 面板停靠的区域，默认为左侧
 * @param floating 面板是否浮动，默认为 true
 * @return 新创建的 PanelDockWidget 指针，如果创建失败返回 nullptr
 */
rviz_common::PanelDockWidget * DummyWindowManager::addPane(
    const QString & name,
    QWidget * pane,
    Qt::DockWidgetArea area,
    bool floating)
{
    try {
        // 仅传递 name 参数
        rviz_common::PanelDockWidget * dock_widget = new rviz_common::PanelDockWidget(name);
        dock_widget->setContentWidget(pane);
        dock_widget->setAllowedAreas(area);
        dock_widget->setFloating(floating);
        // 这里可以添加将 dock_widget 添加到布局中的逻辑
        return dock_widget;
    } catch (const std::bad_alloc& e) {
        std::cerr << "Failed to allocate memory for PanelDockWidget: " << e.what() << std::endl;
        return nullptr;
    }
}

/**
 * @brief 设置状态栏显示的消息
 * @param message 要显示的消息
 */
void DummyWindowManager::setStatus(const QString & message)
{
    // 假设这里有一个状态栏指针 status_bar_
    //status_bar_->showMessage(message);
    std::cout << "Status message: " << message.toStdString() << std::endl;
}
