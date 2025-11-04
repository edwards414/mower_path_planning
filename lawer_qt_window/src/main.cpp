#include "mainwindow.h"
#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QStandardPaths>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    // 初始化ROS2 (在QApplication之前)
    rclcpp::init(argc, argv);
    
    // 创建Qt应用程序
    QApplication app(argc, argv);
    
    // 设置应用程序信息
    app.setApplicationName("Lawer Qt Window");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("Lawer Robotics");
    app.setOrganizationDomain("lawer.robotics");
    
    // 设置应用程序样式
    app.setStyle(QStyleFactory::create("Fusion"));
    
    // 设置深色主题
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    app.setPalette(darkPalette);
    
    // 设置样式表
    app.setStyleSheet(
        "QToolTip {"
        "    color: #ffffff;"
        "    background-color: #2a82da;"
        "    border: 1px solid white;"
        "}"
        "QGroupBox {"
        "    font-weight: bold;"
        "    border: 2px solid #555555;"
        "    border-radius: 5px;"
        "    margin-top: 1ex;"
        "    padding-top: 10px;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 10px;"
        "    padding: 0 5px 0 5px;"
        "}"
        "QScrollBar:vertical {"
        "    background: #2b2b2b;"
        "    width: 15px;"
        "    margin: 22px 0 22px 0;"
        "}"
        "QScrollBar::handle:vertical {"
        "    background: #555555;"
        "    min-height: 20px;"
        "    border-radius: 7px;"
        "}"
        "QScrollBar::add-line:vertical {"
        "    background: #2b2b2b;"
        "    height: 20px;"
        "    subcontrol-position: bottom;"
        "    subcontrol-origin: margin;"
        "}"
        "QScrollBar::sub-line:vertical {"
        "    background: #2b2b2b;"
        "    height: 20px;"
        "    subcontrol-position: top;"
        "    subcontrol-origin: margin;"
        "}"
    );
    
    // 创建配置目录
    QString configDir = QStandardPaths::writableLocation(QStandardPaths::ConfigLocation) + "/LawrRobotics";
    QDir().mkpath(configDir);
    
    qDebug() << "Starting Lawer Qt Window...";
    qDebug() << "Qt Version:" << QT_VERSION_STR;
    qDebug() << "Config Directory:" << configDir;
    
    try {
        // 创建主窗口
        MainWindow window;
        
        // 显示主窗口
        window.show();
        
        qDebug() << "Main window created and shown successfully";
        
        // 运行应用程序事件循环
        int result = app.exec();
        
        qDebug() << "Application finished with code:" << result;
        
        // 清理ROS2
        rclcpp::shutdown();
        
        return result;
    }
    catch (const std::exception& e) {
        qCritical() << "Application error:" << e.what();
        
        // 清理ROS2
        rclcpp::shutdown();
        
        return -1;
    }
    catch (...) {
        qCritical() << "Unknown application error occurred";
        
        // 清理ROS2
        rclcpp::shutdown();
        
        return -1;
    }
}
