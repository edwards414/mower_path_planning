#include "MapController.h"
#include <QDebug>

MapController::MapController(QObject *parent)
    : QObject(parent)
    , map_data_model_(nullptr)
    , node_(nullptr)
    , is_initialized_(false)
{
    map_data_model_ = new MapDataModel(this);

    // 连接信号
    connect(map_data_model_, &MapDataModel::mapDataUpdated,
            this, &MapController::onMapDataModelUpdated);
    connect(map_data_model_, &MapDataModel::zoneDataUpdated,
            this, &MapController::onZoneDataModelUpdated);
    connect(map_data_model_, &MapDataModel::markerDataUpdated,
            this, &MapController::onMarkerDataModelUpdated);
}

MapController::~MapController()
{
    shutdown();
}

bool MapController::initialize(rclcpp::Node::SharedPtr node)
{
    if (is_initialized_) {
        return true;
    }

    if (!node) {
        qDebug() << "MapController: Invalid node provided";
        emit initializationCompleted(false);
        return false;
    }

    node_ = node;
    
    try {
        setupSubscribers();
        is_initialized_ = true;
        
        qDebug() << "MapController initialized successfully";
        emit initializationCompleted(true);
        return true;
    }
    catch (const std::exception& e) {
        qDebug() << "MapController initialization failed:" << e.what();
        is_initialized_ = false;
        emit initializationCompleted(false);
        return false;
    }
}

void MapController::shutdown()
{
    if (is_initialized_) {
        qDebug() << "Shutting down MapController...";
        
        map_subscribers_.clear();
        marker_subscribers_.clear();
        node_.reset();
        is_initialized_ = false;
    }
}

void MapController::setupSubscribers()
{
    if (!node_) return;

    // 创建地图订阅者 (基于RViz配置)
    createMapSubscriber("chennal_map", "/chennal_map");
    createMapSubscriber("free_space", "/free_space");
    createMapSubscriber("free_space_inflated", "/free_space_inflated");
    createMapSubscriber("risk_map", "/risk_map");
    createMapSubscriber("risk_map_inflated", "/risk_map_inflated");
    createMapSubscriber("chennal_map_inflated", "/chennal_map_inflated");
    createMapSubscriber("local_costmap", "/local_costmap/costmap");
    createMapSubscriber("global_costmap", "/global_costmap/costmap");

    // 创建标记订阅者
    createMarkerSubscriber("zone_markers", "/zone_list");
    createMarkerSubscriber("risk_zone_markers", "/risk_zone_list");

    qDebug() << "MapController subscribers created:"
             << "Maps:" << map_subscribers_.size()
             << "Markers:" << marker_subscribers_.size();
}

void MapController::createMapSubscriber(const QString& map_type, const QString& topic_name)
{
    auto subscriber = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_name.toStdString(), 10,
        [this, map_type](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            mapCallback(msg, map_type);
        });
    
    map_subscribers_[map_type] = subscriber;
    qDebug() << "Created map subscriber for:" << map_type << "Topic:" << topic_name;
}

void MapController::createMarkerSubscriber(const QString& marker_type, const QString& topic_name)
{
    auto subscriber = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        topic_name.toStdString(), 10,
        [this, marker_type](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            if (marker_type == "zone_markers") {
                zoneMarkersCallback(msg);
            } else if (marker_type == "risk_zone_markers") {
                riskZoneMarkersCallback(msg);
            }
        });
    
    marker_subscribers_[marker_type] = subscriber;
    qDebug() << "Created marker subscriber for:" << marker_type << "Topic:" << topic_name;
}

void MapController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const QString& map_type)
{
    if (!map_data_model_) return;
    
    map_data_model_->setMapData(map_type, *msg);
    qDebug() << "Received map data for:" << map_type 
             << "Size:" << msg->info.width << "x" << msg->info.height;
}

void MapController::zoneMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    if (!map_data_model_) return;
    
    map_data_model_->setZoneMarkers(*msg);
    qDebug() << "Received zone markers, count:" << msg->markers.size();
}

void MapController::riskZoneMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    if (!map_data_model_) return;
    
    map_data_model_->setRiskZoneMarkers(*msg);
    qDebug() << "Received risk zone markers, count:" << msg->markers.size();
}

void MapController::setMapEnabled(const QString& map_type, bool enabled)
{
    if (map_data_model_) {
        map_data_model_->setMapEnabled(map_type, enabled);
    }
}

void MapController::setMapAlpha(const QString& map_type, float alpha)
{
    if (map_data_model_) {
        map_data_model_->setMapAlpha(map_type, alpha);
    }
}

void MapController::setMapColorScheme(const QString& map_type, const QString& color_scheme)
{
    if (map_data_model_) {
        map_data_model_->setMapColorScheme(map_type, color_scheme);
    }
}

QPointF MapController::mapToWidget(const QPointF& map_point, const QSize& widget_size) const
{
    if (map_data_model_) {
        return map_data_model_->mapToWidget(map_point, widget_size);
    }
    return QPointF(0, 0);
}

QPointF MapController::widgetToMap(const QPointF& widget_point, const QSize& widget_size) const
{
    if (map_data_model_) {
        return map_data_model_->widgetToMap(widget_point, widget_size);
    }
    return QPointF(0, 0);
}

bool MapController::isInitialized() const
{
    return is_initialized_;
}

void MapController::onMapDataModelUpdated(const QString& map_type)
{
    emit mapDataUpdated(map_type);
}

void MapController::onZoneDataModelUpdated()
{
    emit zoneDataUpdated();
}

void MapController::onMarkerDataModelUpdated()
{
    emit markerDataUpdated();
}
