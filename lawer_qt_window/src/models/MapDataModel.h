#ifndef MAPDATAMODEL_H
#define MAPDATAMODEL_H

#include <QObject>
#include <QPointF>
#include <QPolygonF>
#include <QColor>
#include <QVector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

class MapDataModel : public QObject
{
    Q_OBJECT

public:
    explicit MapDataModel(QObject *parent = nullptr);

    // 地图数据结构
    struct MapData {
        int width;
        int height;
        float resolution;
        QPointF origin;
        QVector<int8_t> data;
        QString frame_id;
        QColor color_scheme;
        float alpha;
        bool enabled;
    };

    // 区域数据结构
    struct ZoneData {
        QString id;
        QString name;
        QPolygonF polygon;
        QColor color;
        bool is_risk_zone;
        bool enabled;
    };

    // 标记数据结构
    struct MarkerData {
        QString id;
        QString namespace_name;
        QPointF position;
        QColor color;
        QString text;
        float scale;
        bool enabled;
    };

    // 获取地图数据
    const MapData& getChennalMap() const { return chennal_map_; }
    const MapData& getFreeSpace() const { return free_space_; }
    const MapData& getFreeSpaceInflated() const { return free_space_inflated_; }
    const MapData& getRiskMap() const { return risk_map_; }
    const MapData& getRiskMapInflated() const { return risk_map_inflated_; }
    const MapData& getChennalMapInflated() const { return chennal_map_inflated_; }
    const MapData& getLocalCostmap() const { return local_costmap_; }
    const MapData& getGlobalCostmap() const { return global_costmap_; }

    // 获取区域数据
    const QVector<ZoneData>& getZoneList() const { return zone_list_; }
    const QVector<ZoneData>& getRiskZoneList() const { return risk_zone_list_; }

    // 获取标记数据
    const QVector<MarkerData>& getZoneMarkers() const { return zone_markers_; }
    const QVector<MarkerData>& getRiskZoneMarkers() const { return risk_zone_markers_; }

    // 设置地图数据
    void setMapData(const QString& map_type, const nav_msgs::msg::OccupancyGrid& map_msg);
    void setZoneMarkers(const visualization_msgs::msg::MarkerArray& markers);
    void setRiskZoneMarkers(const visualization_msgs::msg::MarkerArray& markers);

    // 地图显示控制
    void setMapEnabled(const QString& map_type, bool enabled);
    void setMapAlpha(const QString& map_type, float alpha);
    void setMapColorScheme(const QString& map_type, const QString& color_scheme);

    // 坐标转换
    QPointF mapToWidget(const QPointF& map_point, const QSize& widget_size) const;
    QPointF widgetToMap(const QPointF& widget_point, const QSize& widget_size) const;

signals:
    void mapDataUpdated(const QString& map_type);
    void zoneDataUpdated();
    void markerDataUpdated();

private:
    // 地图数据
    MapData chennal_map_;
    MapData free_space_;
    MapData free_space_inflated_;
    MapData risk_map_;
    MapData risk_map_inflated_;
    MapData chennal_map_inflated_;
    MapData local_costmap_;
    MapData global_costmap_;

    // 区域数据
    QVector<ZoneData> zone_list_;
    QVector<ZoneData> risk_zone_list_;

    // 标记数据
    QVector<MarkerData> zone_markers_;
    QVector<MarkerData> risk_zone_markers_;

    // 私有方法
    MapData& getMapDataRef(const QString& map_type);
    QColor getColorFromScheme(const QString& scheme, int8_t value) const;
    void initializeMapData(MapData& map_data);
};

#endif // MAPDATAMODEL_H
