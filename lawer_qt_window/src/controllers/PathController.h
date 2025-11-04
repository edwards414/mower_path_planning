#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "../models/PathDataModel.h"

class PathController : public QObject
{
    Q_OBJECT

public:
    explicit PathController(QObject *parent = nullptr);
    ~PathController();

    // 初始化和清理
    bool initialize(rclcpp::Node::SharedPtr node);
    void shutdown();

    // 获取路径数据模型
    PathDataModel* getPathDataModel() const { return path_data_model_; }

    // 路径控制
    void setPathEnabled(const QString& path_name, bool enabled);
    void setPathColor(const QString& path_name, const QColor& color);
    void setPathLineWidth(const QString& path_name, float width);
    void setPathLineStyle(const QString& path_name, const QString& style);
    void setPathShowArrows(const QString& path_name, bool show_arrows);

    // 路径操作
    void clearPath(const QString& path_name);
    void clearAllPaths();

    // 状态查询
    bool isInitialized() const;

signals:
    void pathDataUpdated(const QString& path_name);
    void waypointDataUpdated();
    void pathCleared(const QString& path_name);
    void initializationCompleted(bool success);

private slots:
    void onPathDataModelUpdated(const QString& path_name);
    void onWaypointDataModelUpdated();
    void onPathDataModelCleared(const QString& path_name);

private:
    PathDataModel* path_data_model_;
    rclcpp::Node::SharedPtr node_;
    bool is_initialized_;

    // ROS2 订阅者
    std::map<QString, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subscribers_;
    std::map<QString, rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr> waypoint_subscribers_;

    // 私有方法
    void setupSubscribers();
    void createPathSubscriber(const QString& path_name, const QString& topic_name);
    void createWaypointSubscriber(const QString& waypoint_type, const QString& topic_name);

    // 回调函数
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg, const QString& path_name);
    void waypointsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void chennalPathArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void coveragePathMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
};

#endif // PATHCONTROLLER_H
