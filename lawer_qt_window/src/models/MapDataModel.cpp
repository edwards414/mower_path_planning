#include "MapDataModel.h"
#include <QDebug>
#include <cmath>

MapDataModel::MapDataModel(QObject *parent)
    : QObject(parent)
{
    // 初始化地图数据
    initializeMapData(chennal_map_);
    initializeMapData(free_space_);
    initializeMapData(free_space_inflated_);
    initializeMapData(risk_map_);
    initializeMapData(risk_map_inflated_);
    initializeMapData(chennal_map_inflated_);
    initializeMapData(local_costmap_);
    initializeMapData(global_costmap_);
}

void MapDataModel::initializeMapData(MapData& map_data)
{
    map_data.width = 0;
    map_data.height = 0;
    map_data.resolution = 0.05f;  // 默认分辨率
    map_data.origin = QPointF(0.0, 0.0);
    map_data.frame_id = "map";
    map_data.color_scheme = QColor(255, 255, 255);
    map_data.alpha = 1.0f;
    map_data.enabled = true;
}

void MapDataModel::setMapData(const QString& map_type, const nav_msgs::msg::OccupancyGrid& map_msg)
{
    MapData& map_data = getMapDataRef(map_type);
    
    map_data.width = map_msg.info.width;
    map_data.height = map_msg.info.height;
    map_data.resolution = map_msg.info.resolution;
    map_data.origin = QPointF(map_msg.info.origin.position.x, map_msg.info.origin.position.y);
    map_data.frame_id = QString::fromStdString(map_msg.header.frame_id);
    
    // 转换地图数据
    map_data.data.clear();
    map_data.data.reserve(map_msg.data.size());
    for (const auto& cell : map_msg.data) {
        map_data.data.append(cell);
    }
    
    emit mapDataUpdated(map_type);
    qDebug() << "Map data updated for:" << map_type 
             << "Size:" << map_data.width << "x" << map_data.height;
}

MapDataModel::MapData& MapDataModel::getMapDataRef(const QString& map_type)
{
    if (map_type == "chennal_map") return chennal_map_;
    else if (map_type == "free_space") return free_space_;
    else if (map_type == "free_space_inflated") return free_space_inflated_;
    else if (map_type == "risk_map") return risk_map_;
    else if (map_type == "risk_map_inflated") return risk_map_inflated_;
    else if (map_type == "chennal_map_inflated") return chennal_map_inflated_;
    else if (map_type == "local_costmap") return local_costmap_;
    else if (map_type == "global_costmap") return global_costmap_;
    else {
        qWarning() << "Unknown map type:" << map_type;
        return chennal_map_;  // 默认返回
    }
}

void MapDataModel::setZoneMarkers(const visualization_msgs::msg::MarkerArray& markers)
{
    zone_markers_.clear();
    
    for (const auto& marker : markers.markers) {
        MarkerData marker_data;
        marker_data.id = QString::fromStdString(marker.ns + "_" + std::to_string(marker.id));
        marker_data.namespace_name = QString::fromStdString(marker.ns);
        marker_data.position = QPointF(marker.pose.position.x, marker.pose.position.y);
        marker_data.color = QColor(
            static_cast<int>(marker.color.r * 255),
            static_cast<int>(marker.color.g * 255),
            static_cast<int>(marker.color.b * 255),
            static_cast<int>(marker.color.a * 255)
        );
        marker_data.text = QString::fromStdString(marker.text);
        marker_data.scale = marker.scale.x;
        marker_data.enabled = true;
        
        zone_markers_.append(marker_data);
    }
    
    emit markerDataUpdated();
    qDebug() << "Zone markers updated, count:" << zone_markers_.size();
}

void MapDataModel::setRiskZoneMarkers(const visualization_msgs::msg::MarkerArray& markers)
{
    risk_zone_markers_.clear();
    
    for (const auto& marker : markers.markers) {
        MarkerData marker_data;
        marker_data.id = QString::fromStdString(marker.ns + "_" + std::to_string(marker.id));
        marker_data.namespace_name = QString::fromStdString(marker.ns);
        marker_data.position = QPointF(marker.pose.position.x, marker.pose.position.y);
        marker_data.color = QColor(
            static_cast<int>(marker.color.r * 255),
            static_cast<int>(marker.color.g * 255),
            static_cast<int>(marker.color.b * 255),
            static_cast<int>(marker.color.a * 255)
        );
        marker_data.text = QString::fromStdString(marker.text);
        marker_data.scale = marker.scale.x;
        marker_data.enabled = true;
        
        risk_zone_markers_.append(marker_data);
    }
    
    emit markerDataUpdated();
    qDebug() << "Risk zone markers updated, count:" << risk_zone_markers_.size();
}

void MapDataModel::setMapEnabled(const QString& map_type, bool enabled)
{
    MapData& map_data = getMapDataRef(map_type);
    if (map_data.enabled != enabled) {
        map_data.enabled = enabled;
        emit mapDataUpdated(map_type);
    }
}

void MapDataModel::setMapAlpha(const QString& map_type, float alpha)
{
    MapData& map_data = getMapDataRef(map_type);
    alpha = qBound(0.0f, alpha, 1.0f);
    if (std::abs(map_data.alpha - alpha) > 0.01f) {
        map_data.alpha = alpha;
        emit mapDataUpdated(map_type);
    }
}

void MapDataModel::setMapColorScheme(const QString& map_type, const QString& color_scheme)
{
    MapData& map_data = getMapDataRef(map_type);
    
    // 根据颜色方案设置颜色
    if (color_scheme == "map") {
        map_data.color_scheme = QColor(255, 255, 255);  // 白色为自由空间
    } else if (color_scheme == "costmap") {
        map_data.color_scheme = QColor(0, 0, 255);      // 蓝色为低代价
    } else if (color_scheme == "raw") {
        map_data.color_scheme = QColor(128, 128, 128);  // 灰色
    }
    
    emit mapDataUpdated(map_type);
}

QColor MapDataModel::getColorFromScheme(const QString& scheme, int8_t value) const
{
    if (value == -1) {
        return QColor(128, 128, 128);  // 未知区域为灰色
    }
    
    if (scheme == "map") {
        if (value == 0) return QColor(255, 255, 255);      // 自由空间为白色
        else if (value == 100) return QColor(0, 0, 0);     // 障碍物为黑色
        else return QColor(128, 128, 128);                 // 其他为灰色
    } else if (scheme == "costmap") {
        // 代价地图：蓝色(低代价) -> 红色(高代价)
        float ratio = value / 100.0f;
        int red = static_cast<int>(255 * ratio);
        int blue = static_cast<int>(255 * (1.0f - ratio));
        return QColor(red, 0, blue);
    } else if (scheme == "raw") {
        // 原始数据显示
        int gray = static_cast<int>(255 * (value / 100.0f));
        return QColor(gray, gray, gray);
    }
    
    return QColor(128, 128, 128);  // 默认灰色
}

QPointF MapDataModel::mapToWidget(const QPointF& map_point, const QSize& widget_size) const
{
    // 使用chennal_map作为参考地图进行坐标转换
    const MapData& ref_map = chennal_map_;
    
    if (ref_map.width == 0 || ref_map.height == 0) {
        return QPointF(0, 0);
    }
    
    // 地图坐标转换为像素坐标
    float pixel_x = (map_point.x() - ref_map.origin.x()) / ref_map.resolution;
    float pixel_y = (map_point.y() - ref_map.origin.y()) / ref_map.resolution;
    
    // 像素坐标转换为widget坐标
    float widget_x = pixel_x * widget_size.width() / ref_map.width;
    float widget_y = widget_size.height() - (pixel_y * widget_size.height() / ref_map.height);
    
    return QPointF(widget_x, widget_y);
}

QPointF MapDataModel::widgetToMap(const QPointF& widget_point, const QSize& widget_size) const
{
    // 使用chennal_map作为参考地图进行坐标转换
    const MapData& ref_map = chennal_map_;
    
    if (ref_map.width == 0 || ref_map.height == 0) {
        return QPointF(0, 0);
    }
    
    // widget坐标转换为像素坐标
    float pixel_x = widget_point.x() * ref_map.width / widget_size.width();
    float pixel_y = (widget_size.height() - widget_point.y()) * ref_map.height / widget_size.height();
    
    // 像素坐标转换为地图坐标
    float map_x = ref_map.origin.x() + pixel_x * ref_map.resolution;
    float map_y = ref_map.origin.y() + pixel_y * ref_map.resolution;
    
    return QPointF(map_x, map_y);
}
