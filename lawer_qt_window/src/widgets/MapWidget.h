#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>
#include <QPointF>
#include <QTransform>
#include "../controllers/MapController.h"
#include "../controllers/PathController.h"

class MapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget *parent = nullptr);
    ~MapWidget();

    // 设置控制器
    void setMapController(MapController* controller);
    void setPathController(PathController* controller);

    // 视图控制
    void setCenter(const QPointF& center);
    void setZoom(float zoom);
    void fitToContent();
    void resetView();

    // 显示控制
    void setGridEnabled(bool enabled);
    void setRobotEnabled(bool enabled);
    void setMapLayerEnabled(const QString& layer, bool enabled);
    void setPathLayerEnabled(const QString& path, bool enabled);

    // 坐标转换
    QPointF mapToWidget(const QPointF& map_point) const;
    QPointF widgetToMap(const QPointF& widget_point) const;

signals:
    void mapClicked(const QPointF& map_point);
    void mapDoubleClicked(const QPointF& map_point);
    void viewChanged(const QPointF& center, float zoom);
    void mousePositionChanged(const QPointF& map_point);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void onMapDataUpdated(const QString& map_type);
    void onPathDataUpdated(const QString& path_name);
    void onZoneDataUpdated();
    void onMarkerDataUpdated();
    void onWaypointDataUpdated();
    void onRefreshTimer();

private:
    MapController* map_controller_;
    PathController* path_controller_;

    // 视图变换
    QTransform view_transform_;
    QPointF view_center_;
    float zoom_level_;
    float min_zoom_;
    float max_zoom_;

    // 鼠标交互
    bool is_dragging_;
    QPointF last_mouse_pos_;
    QPointF drag_start_pos_;

    // 显示选项
    bool grid_enabled_;
    bool robot_enabled_;
    QMap<QString, bool> map_layers_enabled_;
    QMap<QString, bool> path_layers_enabled_;

    // 渲染缓存
    QPixmap map_cache_;
    bool cache_valid_;
    QTimer* refresh_timer_;

    // 私有方法
    void setupUI();
    void updateViewTransform();
    void invalidateCache();
    
    // 绘制方法
    void drawBackground(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawMaps(QPainter& painter);
    void drawMap(QPainter& painter, const QString& map_type);
    void drawPaths(QPainter& painter);
    void drawPath(QPainter& painter, const QString& path_name);
    void drawZones(QPainter& painter);
    void drawMarkers(QPainter& painter);
    void drawWaypoints(QPainter& painter);
    void drawRobot(QPainter& painter);
    void drawCoordinateSystem(QPainter& painter);
    void drawStatusInfo(QPainter& painter);
    
    // 辅助方法
    QColor getMapColor(int8_t value, const QString& color_scheme) const;
    void drawArrow(QPainter& painter, const QPointF& start, const QPointF& end, float width = 2.0f);
    void drawMarker(QPainter& painter, const QPointF& position, const QColor& color, float size = 5.0f);
    QRectF getMapBounds() const;
    
    // 常量
    static constexpr float DEFAULT_ZOOM = 1.0f;
    static constexpr float MIN_ZOOM = 0.1f;
    static constexpr float MAX_ZOOM = 10.0f;
    static constexpr float ZOOM_FACTOR = 1.2f;
};

#endif // MAPWIDGET_H
