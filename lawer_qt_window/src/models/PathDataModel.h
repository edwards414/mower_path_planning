#ifndef PATHDATAMODEL_H
#define PATHDATAMODEL_H

#include <QObject>
#include <QPointF>
#include <QColor>
#include <QVector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PathDataModel : public QObject
{
    Q_OBJECT

public:
    explicit PathDataModel(QObject *parent = nullptr);

    // 路径数据结构
    struct PathData {
        QString name;
        QVector<QPointF> points;
        QVector<float> orientations;  // 每个点的朝向角度
        QColor color;
        float line_width;
        QString line_style;  // "Lines", "Billboards"
        bool show_arrows;
        bool enabled;
        QString frame_id;
    };

    // 航点数据结构
    struct WaypointData {
        QString id;
        QPointF position;
        float orientation;
        QColor color;
        QString text;
        bool enabled;
    };

    // 获取路径数据
    const PathData& getGlobalPlan() const { return global_plan_; }
    const PathData& getChennalPath() const { return chennal_path_; }
    const PathData& getCoveragePath() const { return coverage_path_; }
    const PathData& getRecordPath() const { return record_path_; }
    const PathData& getPlan() const { return plan_; }
    const PathData& getSplitPath() const { return split_path_; }

    // 获取航点数据
    const QVector<WaypointData>& getWaypoints() const { return waypoints_; }
    const QVector<WaypointData>& getChennalPathArray() const { return chennal_path_array_; }
    const QVector<WaypointData>& getCoveragePathMarkers() const { return coverage_path_markers_; }

    // 设置路径数据
    void setPathData(const QString& path_name, const nav_msgs::msg::Path& path_msg);
    void setWaypoints(const visualization_msgs::msg::MarkerArray& markers);
    void setChennalPathArray(const visualization_msgs::msg::MarkerArray& markers);
    void setCoveragePathMarkers(const visualization_msgs::msg::MarkerArray& markers);

    // 路径显示控制
    void setPathEnabled(const QString& path_name, bool enabled);
    void setPathColor(const QString& path_name, const QColor& color);
    void setPathLineWidth(const QString& path_name, float width);
    void setPathLineStyle(const QString& path_name, const QString& style);
    void setPathShowArrows(const QString& path_name, bool show_arrows);

    // 路径操作
    void clearPath(const QString& path_name);
    void clearAllPaths();
    int getPathPointCount(const QString& path_name) const;
    QPointF getPathPoint(const QString& path_name, int index) const;

signals:
    void pathDataUpdated(const QString& path_name);
    void waypointDataUpdated();
    void pathCleared(const QString& path_name);

private:
    // 路径数据
    PathData global_plan_;
    PathData chennal_path_;
    PathData coverage_path_;
    PathData record_path_;
    PathData plan_;
    PathData split_path_;

    // 航点数据
    QVector<WaypointData> waypoints_;
    QVector<WaypointData> chennal_path_array_;
    QVector<WaypointData> coverage_path_markers_;

    // 私有方法
    PathData& getPathDataRef(const QString& path_name);
    void initializePathData(PathData& path_data, const QString& name, const QColor& color);
    WaypointData createWaypointFromMarker(const visualization_msgs::msg::Marker& marker);
};

#endif // PATHDATAMODEL_H
