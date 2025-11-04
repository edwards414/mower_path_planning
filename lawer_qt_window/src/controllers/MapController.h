#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "../models/MapDataModel.h"

class MapController : public QObject
{
    Q_OBJECT

public:
    explicit MapController(QObject *parent = nullptr);
    ~MapController();

    // 初始化和清理
    bool initialize(rclcpp::Node::SharedPtr node);
    void shutdown();

    // 获取地图数据模型
    MapDataModel* getMapDataModel() const { return map_data_model_; }

    // 地图控制
    void setMapEnabled(const QString& map_type, bool enabled);
    void setMapAlpha(const QString& map_type, float alpha);
    void setMapColorScheme(const QString& map_type, const QString& color_scheme);

    // 坐标转换
    QPointF mapToWidget(const QPointF& map_point, const QSize& widget_size) const;
    QPointF widgetToMap(const QPointF& widget_point, const QSize& widget_size) const;

    // 状态查询
    bool isInitialized() const;

signals:
    void mapDataUpdated(const QString& map_type);
    void zoneDataUpdated();
    void markerDataUpdated();
    void initializationCompleted(bool success);

private slots:
    void onMapDataModelUpdated(const QString& map_type);
    void onZoneDataModelUpdated();
    void onMarkerDataModelUpdated();

private:
    MapDataModel* map_data_model_;
    rclcpp::Node::SharedPtr node_;
    bool is_initialized_;

    // ROS2 订阅者
    std::map<QString, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subscribers_;
    std::map<QString, rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_subscribers_;

    // 私有方法
    void setupSubscribers();
    void createMapSubscriber(const QString& map_type, const QString& topic_name);
    void createMarkerSubscriber(const QString& marker_type, const QString& topic_name);

    // 回调函数
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const QString& map_type);
    void zoneMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void riskZoneMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
};

#endif // MAPCONTROLLER_H
