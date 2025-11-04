#include "PathController.h"
#include <QDebug>

PathController::PathController(QObject *parent)
    : QObject(parent)
    , path_data_model_(nullptr)
    , node_(nullptr)
    , is_initialized_(false)
{
    path_data_model_ = new PathDataModel(this);

    // 连接信号
    connect(path_data_model_, &PathDataModel::pathDataUpdated,
            this, &PathController::onPathDataModelUpdated);
    connect(path_data_model_, &PathDataModel::waypointDataUpdated,
            this, &PathController::onWaypointDataModelUpdated);
    connect(path_data_model_, &PathDataModel::pathCleared,
            this, &PathController::onPathDataModelCleared);
}

PathController::~PathController()
{
    shutdown();
}

bool PathController::initialize(rclcpp::Node::SharedPtr node)
{
    if (is_initialized_) {
        return true;
    }

    if (!node) {
        qDebug() << "PathController: Invalid node provided";
        emit initializationCompleted(false);
        return false;
    }

    node_ = node;
    
    try {
        setupSubscribers();
        is_initialized_ = true;
        
        qDebug() << "PathController initialized successfully";
        emit initializationCompleted(true);
        return true;
    }
    catch (const std::exception& e) {
        qDebug() << "PathController initialization failed:" << e.what();
        is_initialized_ = false;
        emit initializationCompleted(false);
        return false;
    }
}

void PathController::shutdown()
{
    if (is_initialized_) {
        qDebug() << "Shutting down PathController...";
        
        path_subscribers_.clear();
        waypoint_subscribers_.clear();
        node_.reset();
        is_initialized_ = false;
    }
}

void PathController::setupSubscribers()
{
    if (!node_) return;

    // 创建路径订阅者 (基于RViz配置)
    createPathSubscriber("global_plan", "/transformed_global_plan");
    createPathSubscriber("chennal_path", "/chennal_path");
    createPathSubscriber("coverage_path", "/coverage_path");
    createPathSubscriber("record_path", "/recorded_path");
    createPathSubscriber("plan", "/plan");
    createPathSubscriber("split_path", "/split_path");

    // 创建航点订阅者
    createWaypointSubscriber("waypoints", "/waypoints");
    createWaypointSubscriber("chennal_path_array", "/chennal_path_array");
    createWaypointSubscriber("coverage_path_markers", "/coverage_path_markers");

    qDebug() << "PathController subscribers created:"
             << "Paths:" << path_subscribers_.size()
             << "Waypoints:" << waypoint_subscribers_.size();
}

void PathController::createPathSubscriber(const QString& path_name, const QString& topic_name)
{
    auto subscriber = node_->create_subscription<nav_msgs::msg::Path>(
        topic_name.toStdString(), 10,
        [this, path_name](const nav_msgs::msg::Path::SharedPtr msg) {
            pathCallback(msg, path_name);
        });
    
    path_subscribers_[path_name] = subscriber;
    qDebug() << "Created path subscriber for:" << path_name << "Topic:" << topic_name;
}

void PathController::createWaypointSubscriber(const QString& waypoint_type, const QString& topic_name)
{
    auto subscriber = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        topic_name.toStdString(), 10,
        [this, waypoint_type](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            if (waypoint_type == "waypoints") {
                waypointsCallback(msg);
            } else if (waypoint_type == "chennal_path_array") {
                chennalPathArrayCallback(msg);
            } else if (waypoint_type == "coverage_path_markers") {
                coveragePathMarkersCallback(msg);
            }
        });
    
    waypoint_subscribers_[waypoint_type] = subscriber;
    qDebug() << "Created waypoint subscriber for:" << waypoint_type << "Topic:" << topic_name;
}

void PathController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg, const QString& path_name)
{
    if (!path_data_model_) return;
    
    path_data_model_->setPathData(path_name, *msg);
    qDebug() << "Received path data for:" << path_name 
             << "Points:" << msg->poses.size();
}

void PathController::waypointsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    if (!path_data_model_) return;
    
    path_data_model_->setWaypoints(*msg);
    qDebug() << "Received waypoints, count:" << msg->markers.size();
}

void PathController::chennalPathArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    if (!path_data_model_) return;
    
    path_data_model_->setChennalPathArray(*msg);
    qDebug() << "Received chennal path array, count:" << msg->markers.size();
}

void PathController::coveragePathMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    if (!path_data_model_) return;
    
    path_data_model_->setCoveragePathMarkers(*msg);
    qDebug() << "Received coverage path markers, count:" << msg->markers.size();
}

void PathController::setPathEnabled(const QString& path_name, bool enabled)
{
    if (path_data_model_) {
        path_data_model_->setPathEnabled(path_name, enabled);
    }
}

void PathController::setPathColor(const QString& path_name, const QColor& color)
{
    if (path_data_model_) {
        path_data_model_->setPathColor(path_name, color);
    }
}

void PathController::setPathLineWidth(const QString& path_name, float width)
{
    if (path_data_model_) {
        path_data_model_->setPathLineWidth(path_name, width);
    }
}

void PathController::setPathLineStyle(const QString& path_name, const QString& style)
{
    if (path_data_model_) {
        path_data_model_->setPathLineStyle(path_name, style);
    }
}

void PathController::setPathShowArrows(const QString& path_name, bool show_arrows)
{
    if (path_data_model_) {
        path_data_model_->setPathShowArrows(path_name, show_arrows);
    }
}

void PathController::clearPath(const QString& path_name)
{
    if (path_data_model_) {
        path_data_model_->clearPath(path_name);
    }
}

void PathController::clearAllPaths()
{
    if (path_data_model_) {
        path_data_model_->clearAllPaths();
    }
}

bool PathController::isInitialized() const
{
    return is_initialized_;
}

void PathController::onPathDataModelUpdated(const QString& path_name)
{
    emit pathDataUpdated(path_name);
}

void PathController::onWaypointDataModelUpdated()
{
    emit waypointDataUpdated();
}

void PathController::onPathDataModelCleared(const QString& path_name)
{
    emit pathCleared(path_name);
}
