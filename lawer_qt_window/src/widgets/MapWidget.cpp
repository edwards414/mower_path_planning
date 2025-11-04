#include "MapWidget.h"
#include <QDebug>
#include <QPaintEvent>
#include <QApplication>
#include <cmath>

MapWidget::MapWidget(QWidget *parent)
    : QWidget(parent)
    , map_controller_(nullptr)
    , path_controller_(nullptr)
    , view_center_(0, 0)
    , zoom_level_(DEFAULT_ZOOM)
    , min_zoom_(MIN_ZOOM)
    , max_zoom_(MAX_ZOOM)
    , is_dragging_(false)
    , grid_enabled_(true)
    , robot_enabled_(true)
    , cache_valid_(false)
    , refresh_timer_(new QTimer(this))
{
    setupUI();
}

MapWidget::~MapWidget()
{
}

void MapWidget::setupUI()
{
    setMinimumSize(400, 300);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    
    // 设置背景色和样式
    setStyleSheet(
        "MapWidget {"
        "  background-color: #2b2b2b;"
        "  border: 1px solid #555555;"
        "}"
    );
    
    // 初始化显示选项
    map_layers_enabled_["chennal_map"] = true;
    map_layers_enabled_["free_space"] = true;
    map_layers_enabled_["free_space_inflated"] = false;
    map_layers_enabled_["risk_map"] = true;
    map_layers_enabled_["risk_map_inflated"] = false;
    map_layers_enabled_["chennal_map_inflated"] = false;
    map_layers_enabled_["local_costmap"] = true;
    map_layers_enabled_["global_costmap"] = true;
    
    path_layers_enabled_["global_plan"] = true;
    path_layers_enabled_["chennal_path"] = true;
    path_layers_enabled_["coverage_path"] = true;
    path_layers_enabled_["record_path"] = true;
    path_layers_enabled_["plan"] = true;
    path_layers_enabled_["split_path"] = true;
    
    // 设置刷新定时器
    refresh_timer_->setSingleShot(true);
    refresh_timer_->setInterval(16);  // ~60 FPS
    connect(refresh_timer_, &QTimer::timeout, this, QOverload<>::of(&MapWidget::update));
    
    updateViewTransform();
}

void MapWidget::setMapController(MapController* controller)
{
    if (map_controller_) {
        disconnect(map_controller_, nullptr, this, nullptr);
    }
    
    map_controller_ = controller;
    
    if (map_controller_) {
        connect(map_controller_, &MapController::mapDataUpdated,
                this, &MapWidget::onMapDataUpdated);
        connect(map_controller_, &MapController::zoneDataUpdated,
                this, &MapWidget::onZoneDataUpdated);
        connect(map_controller_, &MapController::markerDataUpdated,
                this, &MapWidget::onMarkerDataUpdated);
    }
    
    invalidateCache();
}

void MapWidget::setPathController(PathController* controller)
{
    if (path_controller_) {
        disconnect(path_controller_, nullptr, this, nullptr);
    }
    
    path_controller_ = controller;
    
    if (path_controller_) {
        connect(path_controller_, &PathController::pathDataUpdated,
                this, &MapWidget::onPathDataUpdated);
        connect(path_controller_, &PathController::waypointDataUpdated,
                this, &MapWidget::onWaypointDataUpdated);
    }
    
    invalidateCache();
}

void MapWidget::setCenter(const QPointF& center)
{
    view_center_ = center;
    updateViewTransform();
    invalidateCache();
}

void MapWidget::setZoom(float zoom)
{
    zoom_level_ = qBound(min_zoom_, zoom, max_zoom_);
    updateViewTransform();
    invalidateCache();
}

void MapWidget::fitToContent()
{
    QRectF bounds = getMapBounds();
    if (bounds.isEmpty()) {
        return;
    }
    
    // 计算适合的缩放级别
    float zoom_x = width() / bounds.width();
    float zoom_y = height() / bounds.height();
    float zoom = qMin(zoom_x, zoom_y) * 0.9f;  // 留一些边距
    
    setZoom(zoom);
    setCenter(bounds.center());
    
    emit viewChanged(view_center_, zoom_level_);
}

void MapWidget::resetView()
{
    setCenter(QPointF(0, 0));
    setZoom(DEFAULT_ZOOM);
    emit viewChanged(view_center_, zoom_level_);
}

void MapWidget::updateViewTransform()
{
    view_transform_ = QTransform();
    view_transform_.translate(width() / 2.0, height() / 2.0);
    view_transform_.scale(zoom_level_, -zoom_level_);  // Y轴翻转
    view_transform_.translate(-view_center_.x(), -view_center_.y());
}

void MapWidget::invalidateCache()
{
    cache_valid_ = false;
    if (!refresh_timer_->isActive()) {
        refresh_timer_->start();
    }
}

QPointF MapWidget::mapToWidget(const QPointF& map_point) const
{
    return view_transform_.map(map_point);
}

QPointF MapWidget::widgetToMap(const QPointF& widget_point) const
{
    return view_transform_.inverted().map(widget_point);
}

void MapWidget::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    
    // 绘制背景
    painter.fillRect(rect(), QColor(43, 43, 43));  // 深灰色背景
    
    // 绘制边框
    painter.setPen(QPen(QColor(85, 85, 85), 1));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));
    
    // 设置变换
    painter.setTransform(view_transform_);
    
    // 绘制各个层
    drawBackground(painter);
    
    if (grid_enabled_) {
        drawGrid(painter);
    }
    
    drawMaps(painter);
    drawZones(painter);
    drawPaths(painter);
    drawMarkers(painter);
    drawWaypoints(painter);
    
    if (robot_enabled_) {
        drawRobot(painter);
    }
    
    // 重置变换绘制UI元素
    painter.resetTransform();
    drawCoordinateSystem(painter);
    
    // 绘制状态信息
    drawStatusInfo(painter);
}

void MapWidget::drawBackground(QPainter& painter)
{
    // 绘制背景网格或纹理
    painter.fillRect(getMapBounds(), QColor(50, 50, 50, 100));
}

void MapWidget::drawGrid(QPainter& painter)
{
    // 获取当前视图范围
    QPointF top_left = widgetToMap(QPointF(0, 0));
    QPointF bottom_right = widgetToMap(QPointF(width(), height()));
    
    QRectF view_bounds(top_left.x(), bottom_right.y(), 
                       bottom_right.x() - top_left.x(), 
                       top_left.y() - bottom_right.y());
    
    // 扩展边界以确保有足够的网格显示
    view_bounds = view_bounds.adjusted(-10, -10, 10, 10);
    
    painter.setPen(QPen(QColor(100, 100, 100, 150), 0));
    
    // 计算网格间距
    float grid_size = 1.0f;
    while (grid_size * zoom_level_ < 20) grid_size *= 2;
    while (grid_size * zoom_level_ > 100) grid_size /= 2;
    
    // 绘制垂直线
    float start_x = std::floor(view_bounds.left() / grid_size) * grid_size;
    for (float x = start_x; x <= view_bounds.right(); x += grid_size) {
        painter.drawLine(QPointF(x, view_bounds.top()), QPointF(x, view_bounds.bottom()));
    }
    
    // 绘制水平线
    float start_y = std::floor(view_bounds.bottom() / grid_size) * grid_size;
    for (float y = start_y; y <= view_bounds.top(); y += grid_size) {
        painter.drawLine(QPointF(view_bounds.left(), y), QPointF(view_bounds.right(), y));
    }
    
    // 绘制坐标轴 (更醒目)
    painter.setPen(QPen(QColor(200, 200, 200, 200), 0));
    painter.drawLine(QPointF(view_bounds.left(), 0), QPointF(view_bounds.right(), 0));
    painter.drawLine(QPointF(0, view_bounds.bottom()), QPointF(0, view_bounds.top()));
    
    // 绘制原点标记
    painter.setPen(QPen(Qt::red, 0));
    painter.setBrush(Qt::red);
    painter.drawEllipse(QPointF(0, 0), 0.1, 0.1);
}

void MapWidget::drawMaps(QPainter& painter)
{
    if (!map_controller_ || !map_controller_->getMapDataModel()) {
        return;
    }
    
    // 按层次顺序绘制地图
    QStringList map_order = {
        "chennal_map", "free_space", "risk_map", 
        "chennal_map_inflated", "free_space_inflated", "risk_map_inflated",
        "global_costmap", "local_costmap"
    };
    
    for (const QString& map_type : map_order) {
        if (map_layers_enabled_.value(map_type, false)) {
            drawMap(painter, map_type);
        }
    }
}

void MapWidget::drawMap(QPainter& painter, const QString& map_type)
{
    if (!map_controller_) return;
    
    auto model = map_controller_->getMapDataModel();
    if (!model) return;
    
    // 获取对应的地图数据
    const auto* map_data = [&]() -> const MapDataModel::MapData* {
        if (map_type == "chennal_map") return &model->getChennalMap();
        else if (map_type == "free_space") return &model->getFreeSpace();
        else if (map_type == "free_space_inflated") return &model->getFreeSpaceInflated();
        else if (map_type == "risk_map") return &model->getRiskMap();
        else if (map_type == "risk_map_inflated") return &model->getRiskMapInflated();
        else if (map_type == "chennal_map_inflated") return &model->getChennalMapInflated();
        else if (map_type == "local_costmap") return &model->getLocalCostmap();
        else if (map_type == "global_costmap") return &model->getGlobalCostmap();
        return nullptr;
    }();
    
    if (!map_data || map_data->width == 0 || map_data->height == 0) {
        return;
    }
    
    // 创建地图图像
    QImage map_image(map_data->width, map_data->height, QImage::Format_ARGB32);
    
    for (int y = 0; y < map_data->height; ++y) {
        for (int x = 0; x < map_data->width; ++x) {
            int index = y * map_data->width + x;
            if (index < map_data->data.size()) {
                int8_t value = map_data->data[index];
                QColor color = getMapColor(value, map_type);
                color.setAlphaF(map_data->alpha);
                map_image.setPixelColor(x, map_data->height - 1 - y, color);  // Y轴翻转
            }
        }
    }
    
    // 绘制地图
    QRectF map_rect(
        map_data->origin.x(),
        map_data->origin.y(),
        map_data->width * map_data->resolution,
        map_data->height * map_data->resolution
    );
    
    painter.drawImage(map_rect, map_image);
}

void MapWidget::drawPaths(QPainter& painter)
{
    if (!path_controller_ || !path_controller_->getPathDataModel()) {
        return;
    }
    
    QStringList path_order = {
        "global_plan", "plan", "coverage_path", 
        "chennal_path", "record_path", "split_path"
    };
    
    for (const QString& path_name : path_order) {
        if (path_layers_enabled_.value(path_name, false)) {
            drawPath(painter, path_name);
        }
    }
}

void MapWidget::drawPath(QPainter& painter, const QString& path_name)
{
    if (!path_controller_) return;
    
    auto model = path_controller_->getPathDataModel();
    if (!model) return;
    
    // 获取对应的路径数据
    const auto* path_data = [&]() -> const PathDataModel::PathData* {
        if (path_name == "global_plan") return &model->getGlobalPlan();
        else if (path_name == "chennal_path") return &model->getChennalPath();
        else if (path_name == "coverage_path") return &model->getCoveragePath();
        else if (path_name == "record_path") return &model->getRecordPath();
        else if (path_name == "plan") return &model->getPlan();
        else if (path_name == "split_path") return &model->getSplitPath();
        return nullptr;
    }();
    
    if (!path_data || path_data->points.isEmpty()) {
        return;
    }
    
    // 设置画笔
    QPen pen(path_data->color, path_data->line_width);
    if (path_data->line_style == "Billboards") {
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
    }
    painter.setPen(pen);
    
    // 绘制路径线
    for (int i = 1; i < path_data->points.size(); ++i) {
        painter.drawLine(path_data->points[i-1], path_data->points[i]);
    }
    
    // 绘制箭头
    if (path_data->show_arrows && path_data->points.size() > 1) {
        painter.setBrush(path_data->color);
        for (int i = 1; i < path_data->points.size(); i += 5) {  // 每5个点绘制一个箭头
            if (i < path_data->orientations.size()) {
                QPointF pos = path_data->points[i];
                float angle = path_data->orientations[i];
                
                // 绘制箭头
                painter.save();
                painter.translate(pos);
                painter.rotate(qRadiansToDegrees(angle));
                
                QPolygonF arrow;
                arrow << QPointF(0.2, 0) << QPointF(-0.1, 0.1) << QPointF(-0.1, -0.1);
                painter.drawPolygon(arrow);
                
                painter.restore();
            }
        }
    }
}

void MapWidget::drawZones(QPainter& painter)
{
    if (!map_controller_ || !map_controller_->getMapDataModel()) {
        return;
    }
    
    auto model = map_controller_->getMapDataModel();
    
    // 绘制普通区域
    const auto& zones = model->getZoneList();
    for (const auto& zone : zones) {
        if (zone.enabled) {
            painter.setPen(QPen(zone.color, 0.05f));
            painter.setBrush(QBrush(zone.color, Qt::Dense6Pattern));
            painter.drawPolygon(zone.polygon);
        }
    }
    
    // 绘制风险区域
    const auto& risk_zones = model->getRiskZoneList();
    for (const auto& zone : risk_zones) {
        if (zone.enabled) {
            painter.setPen(QPen(zone.color, 0.05f));
            painter.setBrush(QBrush(zone.color, Qt::Dense4Pattern));
            painter.drawPolygon(zone.polygon);
        }
    }
}

void MapWidget::drawMarkers(QPainter& painter)
{
    if (!map_controller_ || !map_controller_->getMapDataModel()) {
        return;
    }
    
    auto model = map_controller_->getMapDataModel();
    
    // 绘制区域标记
    const auto& zone_markers = model->getZoneMarkers();
    for (const auto& marker : zone_markers) {
        if (marker.enabled) {
            drawMarker(painter, marker.position, marker.color, marker.scale);
        }
    }
    
    // 绘制风险区域标记
    const auto& risk_markers = model->getRiskZoneMarkers();
    for (const auto& marker : risk_markers) {
        if (marker.enabled) {
            drawMarker(painter, marker.position, marker.color, marker.scale);
        }
    }
}

void MapWidget::drawWaypoints(QPainter& painter)
{
    if (!path_controller_ || !path_controller_->getPathDataModel()) {
        return;
    }
    
    auto model = path_controller_->getPathDataModel();
    
    // 绘制航点
    const auto& waypoints = model->getWaypoints();
    for (const auto& waypoint : waypoints) {
        if (waypoint.enabled) {
            drawMarker(painter, waypoint.position, waypoint.color, 0.3f);
        }
    }
    
    // 绘制覆盖路径标记
    const auto& coverage_markers = model->getCoveragePathMarkers();
    for (const auto& marker : coverage_markers) {
        if (marker.enabled) {
            drawMarker(painter, marker.position, marker.color, 0.2f);
        }
    }
}

void MapWidget::drawRobot(QPainter& painter)
{
    // 绘制机器人位置 (假设在原点)
    painter.setPen(QPen(Qt::red, 0.05f));
    painter.setBrush(Qt::red);
    
    QPointF robot_pos(0, 0);  // 这里应该从TF获取实际位置
    
    // 绘制机器人本体 (圆形)
    painter.drawEllipse(robot_pos, 0.2, 0.2);
    
    // 绘制朝向箭头
    painter.drawLine(robot_pos, robot_pos + QPointF(0.3, 0));
}

void MapWidget::drawCoordinateSystem(QPainter& painter)
{
    // 在右下角绘制坐标系指示器
    painter.setPen(QPen(Qt::white, 2));
    
    QPointF origin(width() - 60, height() - 60);
    QPointF x_axis = origin + QPointF(30, 0);
    QPointF y_axis = origin + QPointF(0, -30);
    
    painter.drawLine(origin, x_axis);
    painter.drawLine(origin, y_axis);
    
    painter.setPen(Qt::red);
    painter.drawText(x_axis + QPointF(5, 5), "X");
    painter.setPen(Qt::green);
    painter.drawText(y_axis + QPointF(-10, -5), "Y");
}

void MapWidget::drawStatusInfo(QPainter& painter)
{
    // 在左上角绘制状态信息
    painter.setPen(QPen(Qt::white, 1));
    painter.setFont(QFont("Arial", 10));
    
    QStringList info;
    info << QString("缩放: %1%").arg(static_cast<int>(zoom_level_ * 100));
    info << QString("中心: (%.1f, %.1f)").arg(view_center_.x()).arg(view_center_.y());
    
    if (!map_controller_ || !map_controller_->isInitialized()) {
        info << "地图: 未连接";
    } else {
        info << "地图: 已连接";
    }
    
    if (!path_controller_ || !path_controller_->isInitialized()) {
        info << "路径: 未连接";
    } else {
        info << "路径: 已连接";
    }
    
    int y = 15;
    for (const QString& line : info) {
        painter.drawText(10, y, line);
        y += 15;
    }
}

void MapWidget::drawMarker(QPainter& painter, const QPointF& position, const QColor& color, float size)
{
    painter.setPen(QPen(color, 0.02f));
    painter.setBrush(color);
    painter.drawEllipse(position, size, size);
}

QColor MapWidget::getMapColor(int8_t value, const QString& map_type) const
{
    if (value == -1) {
        return QColor(128, 128, 128, 100);  // 未知区域
    }
    
    if (map_type.contains("costmap")) {
        // 代价地图：蓝色(低代价) -> 红色(高代价)
        float ratio = value / 100.0f;
        int red = static_cast<int>(255 * ratio);
        int blue = static_cast<int>(255 * (1.0f - ratio));
        return QColor(red, 0, blue);
    } else if (map_type.contains("risk")) {
        // 风险地图：红色系
        if (value == 0) return QColor(255, 255, 255, 50);  // 安全区域
        else return QColor(255, 0, 0, value * 2);  // 风险区域
    } else {
        // 普通地图
        if (value == 0) return QColor(255, 255, 255);      // 自由空间
        else if (value == 100) return QColor(0, 0, 0);     // 障碍物
        else return QColor(128, 128, 128);                 // 其他
    }
}

QRectF MapWidget::getMapBounds() const
{
    if (!map_controller_ || !map_controller_->getMapDataModel()) {
        return QRectF(-10, -10, 20, 20);  // 默认范围
    }
    
    auto model = map_controller_->getMapDataModel();
    const auto& chennal_map = model->getChennalMap();
    
    if (chennal_map.width > 0 && chennal_map.height > 0) {
        return QRectF(
            chennal_map.origin.x(),
            chennal_map.origin.y(),
            chennal_map.width * chennal_map.resolution,
            chennal_map.height * chennal_map.resolution
        );
    }
    
    return QRectF(-10, -10, 20, 20);
}

// 鼠标事件处理
void MapWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        is_dragging_ = true;
        last_mouse_pos_ = event->position();
        drag_start_pos_ = event->position();
        setCursor(Qt::ClosedHandCursor);
    }
}

void MapWidget::mouseMoveEvent(QMouseEvent* event)
{
    QPointF map_pos = widgetToMap(event->position());
    emit mousePositionChanged(map_pos);
    
    if (is_dragging_) {
        QPointF delta = event->position() - last_mouse_pos_;
        QPointF map_delta = QPointF(delta.x() / zoom_level_, delta.y() / zoom_level_);
        
        view_center_ -= map_delta;
        updateViewTransform();
        invalidateCache();
        
        last_mouse_pos_ = event->position();
    }
}

void MapWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && is_dragging_) {
        is_dragging_ = false;
        setCursor(Qt::ArrowCursor);
        
        // 如果只是点击而不是拖拽
        QPointF delta = event->position() - drag_start_pos_;
        if (delta.manhattanLength() < 5) {
            QPointF map_pos = widgetToMap(event->position());
            emit mapClicked(map_pos);
        }
        
        emit viewChanged(view_center_, zoom_level_);
    }
}

void MapWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    QPointF map_pos = widgetToMap(event->position());
    emit mapDoubleClicked(map_pos);
}

void MapWidget::wheelEvent(QWheelEvent* event)
{
    float zoom_factor = (event->angleDelta().y() > 0) ? ZOOM_FACTOR : (1.0f / ZOOM_FACTOR);
    float new_zoom = zoom_level_ * zoom_factor;
    new_zoom = qBound(min_zoom_, new_zoom, max_zoom_);
    
    if (new_zoom != zoom_level_) {
        // 以鼠标位置为中心缩放
        QPointF mouse_map_pos = widgetToMap(event->position());
        
        zoom_level_ = new_zoom;
        updateViewTransform();
        
        QPointF new_mouse_map_pos = widgetToMap(event->position());
        view_center_ += mouse_map_pos - new_mouse_map_pos;
        updateViewTransform();
        
        invalidateCache();
        emit viewChanged(view_center_, zoom_level_);
    }
}

void MapWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    updateViewTransform();
    invalidateCache();
}

// 槽函数
void MapWidget::onMapDataUpdated(const QString& map_type)
{
    Q_UNUSED(map_type)
    invalidateCache();
}

void MapWidget::onPathDataUpdated(const QString& path_name)
{
    Q_UNUSED(path_name)
    invalidateCache();
}

void MapWidget::onZoneDataUpdated()
{
    invalidateCache();
}

void MapWidget::onMarkerDataUpdated()
{
    invalidateCache();
}

void MapWidget::onWaypointDataUpdated()
{
    invalidateCache();
}

void MapWidget::onRefreshTimer()
{
    update();
}

// 显示控制方法
void MapWidget::setGridEnabled(bool enabled)
{
    grid_enabled_ = enabled;
    invalidateCache();
}

void MapWidget::setRobotEnabled(bool enabled)
{
    robot_enabled_ = enabled;
    invalidateCache();
}

void MapWidget::setMapLayerEnabled(const QString& layer, bool enabled)
{
    map_layers_enabled_[layer] = enabled;
    invalidateCache();
}

void MapWidget::setPathLayerEnabled(const QString& path, bool enabled)
{
    path_layers_enabled_[path] = enabled;
    invalidateCache();
}
