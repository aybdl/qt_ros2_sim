
#ifndef DUMMYWINDOWMANAGER_HPP
#define DUMMYWINDOWMANAGER_HPP

#pragma once

#include <rviz_common/window_manager_interface.hpp>
#include <QWidget>
#include <QString>
#include <Qt>

class DummyWindowManager : public rviz_common::WindowManagerInterface
{
public:
    DummyWindowManager(QWidget * parent = nullptr);
    ~DummyWindowManager() override;

    QWidget * getParentWindow() override;

    rviz_common::PanelDockWidget * addPane(
        const QString & name,
        QWidget * pane,
        Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
        bool floating = true) override;

    void setStatus(const QString & message) override;

private:
    QWidget * parent_widget_;
};

#endif // DUMMYWINDOWMANAGER_H
