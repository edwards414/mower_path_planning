#include "PathDataModel.h"
#include <QDebug>
#include <cmath>

PathDataModel::PathDataModel(QObject *parent)
    : QObject(parent)
{
    // 初始化路径数据，使用与RViz配置一致的颜色
    initializePathData(global_plan_, "global_plan", QColor(25, 255, 0));      // 绿色
    initializePathData(chennal_path_, "chennal_path", QColor(87, 227, 137));   // 浅绿色
    initializePathData(coverage_path_, "coverage_path", QColor(25, 255, 0));   // 绿色
    initializePathData(record_path_, "record_path", QColor(229, 165, 10));     // 橙色
    initializePathData(plan_, "plan", QColor(51, 209, 122));                  // 青绿色
    initializePathData(split_path_, "split_path", QColor(25, 255, 255));      // 青色
}

void PathDataModel::initializePathData(PathData& path_data, const QString& name, const QColor& color)
{
    path_data.name = name;
    path_data.color = color;
    path_data.line_width = 0.03f;  // 默认线宽
    path_data.line_style = "Lines";
    path_data.show_arrows = false;
    path_data.enabled = true;
    path_data.frame_id = "map";
    
    // 根据路径类型设置特定属性
    if (name == "chennal_path") {
        path_data.line_style = "Billboards";
        path_data.line_width = 0.08f;
    } else if (name == "coverage_path") {
        path_data.show_arrows = true;
        path_data.line_width = 0.03f;
    } else if (name == "record_path") {
        path_data.line_style = "Billboards";
        path_data.line_width = 0.10f;
    } else if (name == "plan") {
        path_data.show_arrows = true;
        path_data.line_width = 0.03f;
    } else if (name == "split_path") {
        path_data.line_style = "Billboards";
        path_data.line_width = 0.06f;
    }
}

PathDataModel::PathData& PathDataModel::getPathDataRef(const QString& path_name)
{
    if (path_name == "global_plan") return global_plan_;
    else if (path_name == "chennal_path") return chennal_path_;
    else if (path_name == "coverage_path") return coverage_path_;
    else if (path_name == "record_path") return record_path_;
    else if (path_name == "plan") return plan_;
    else if (path_name == "split_path") return split_path_;
    else {
        qWarning() << "Unknown path name:" << path_name;
        return global_plan_;  // 默认返回
    }
}

void PathDataModel::setPathData(const QString& path_name, const nav_msgs::msg::Path& path_msg)
{
    PathData& path_data = getPathDataRef(path_name);
    
    // 清空现有数据
    path_data.points.clear();
    path_data.orientations.clear();
    path_data.frame_id = QString::fromStdString(path_msg.header.frame_id);
    
    // 转换路径点
    for (const auto& pose_stamped : path_msg.poses) {
        const auto& pose = pose_stamped.pose;
        
        // 提取位置
        QPointF point(pose.position.x, pose.position.y);
        path_data.points.append(point);
        
        // 提取朝向角度 (从四元数转换为欧拉角)
        const auto& q = pose.orientation;
        float yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y), 
                              1.0f - 2.0f * (q.y * q.y + q.z * q.z));
        path_data.orientations.append(yaw);
    }
    
    emit pathDataUpdated(path_name);
    qDebug() << "Path data updated for:" << path_name 
             << "Points:" << path_data.points.size();
}

void PathDataModel::setWaypoints(const visualization_msgs::msg::MarkerArray& markers)
{
    waypoints_.clear();
    
    for (const auto& marker : markers.markers) {
        WaypointData waypoint = createWaypointFromMarker(marker);
        waypoints_.append(waypoint);
    }
    
    emit waypointDataUpdated();
    qDebug() << "Waypoints updated, count:" << waypoints_.size();
}

void PathDataModel::setChennalPathArray(const visualization_msgs::msg::MarkerArray& markers)
{
    chennal_path_array_.clear();
    
    for (const auto& marker : markers.markers) {
        WaypointData waypoint = createWaypointFromMarker(marker);
        chennal_path_array_.append(waypoint);
    }
    
    emit waypointDataUpdated();
    qDebug() << "Chennal path array updated, count:" << chennal_path_array_.size();
}

void PathDataModel::setCoveragePathMarkers(const visualization_msgs::msg::MarkerArray& markers)
{
    coverage_path_markers_.clear();
    
    for (const auto& marker : markers.markers) {
        WaypointData waypoint = createWaypointFromMarker(marker);
        coverage_path_markers_.append(waypoint);
    }
    
    emit waypointDataUpdated();
    qDebug() << "Coverage path markers updated, count:" << coverage_path_markers_.size();
}

PathDataModel::WaypointData PathDataModel::createWaypointFromMarker(const visualization_msgs::msg::Marker& marker)
{
    WaypointData waypoint;
    waypoint.id = QString::fromStdString(marker.ns + "_" + std::to_string(marker.id));
    waypoint.position = QPointF(marker.pose.position.x, marker.pose.position.y);
    
    // 计算朝向角度
    const auto& q = marker.pose.orientation;
    waypoint.orientation = std::atan2(2.0f * (q.w * q.z + q.x * q.y), 
                                     1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    
    waypoint.color = QColor(
        static_cast<int>(marker.color.r * 255),
        static_cast<int>(marker.color.g * 255),
        static_cast<int>(marker.color.b * 255),
        static_cast<int>(marker.color.a * 255)
    );
    waypoint.text = QString::fromStdString(marker.text);
    waypoint.enabled = true;
    
    return waypoint;
}

void PathDataModel::setPathEnabled(const QString& path_name, bool enabled)
{
    PathData& path_data = getPathDataRef(path_name);
    if (path_data.enabled != enabled) {
        path_data.enabled = enabled;
        emit pathDataUpdated(path_name);
    }
}

void PathDataModel::setPathColor(const QString& path_name, const QColor& color)
{
    PathData& path_data = getPathDataRef(path_name);
    if (path_data.color != color) {
        path_data.color = color;
        emit pathDataUpdated(path_name);
    }
}

void PathDataModel::setPathLineWidth(const QString& path_name, float width)
{
    PathData& path_data = getPathDataRef(path_name);
    width = qMax(0.01f, width);  // 最小线宽
    if (std::abs(path_data.line_width - width) > 0.001f) {
        path_data.line_width = width;
        emit pathDataUpdated(path_name);
    }
}

void PathDataModel::setPathLineStyle(const QString& path_name, const QString& style)
{
    PathData& path_data = getPathDataRef(path_name);
    if (path_data.line_style != style) {
        path_data.line_style = style;
        emit pathDataUpdated(path_name);
    }
}

void PathDataModel::setPathShowArrows(const QString& path_name, bool show_arrows)
{
    PathData& path_data = getPathDataRef(path_name);
    if (path_data.show_arrows != show_arrows) {
        path_data.show_arrows = show_arrows;
        emit pathDataUpdated(path_name);
    }
}

void PathDataModel::clearPath(const QString& path_name)
{
    PathData& path_data = getPathDataRef(path_name);
    path_data.points.clear();
    path_data.orientations.clear();
    
    emit pathCleared(path_name);
    emit pathDataUpdated(path_name);
    qDebug() << "Path cleared:" << path_name;
}

void PathDataModel::clearAllPaths()
{
    QStringList path_names = {"global_plan", "chennal_path", "coverage_path", 
                             "record_path", "plan", "split_path"};
    
    for (const QString& path_name : path_names) {
        clearPath(path_name);
    }
    
    waypoints_.clear();
    chennal_path_array_.clear();
    coverage_path_markers_.clear();
    
    emit waypointDataUpdated();
    qDebug() << "All paths cleared";
}

int PathDataModel::getPathPointCount(const QString& path_name) const
{
    const PathData& path_data = const_cast<PathDataModel*>(this)->getPathDataRef(path_name);
    return path_data.points.size();
}

QPointF PathDataModel::getPathPoint(const QString& path_name, int index) const
{
    const PathData& path_data = const_cast<PathDataModel*>(this)->getPathDataRef(path_name);
    if (index >= 0 && index < path_data.points.size()) {
        return path_data.points[index];
    }
    return QPointF(0, 0);
}
