#include "ServiceModel.h"
#include <QDebug>
#include <chrono>

using namespace std::chrono_literals;

ServiceModel::ServiceModel(QObject *parent)
    : QObject(parent)
    , node_(nullptr)
    , executor_(nullptr)
    , node_initialized_(false)
{
}

ServiceModel::~ServiceModel()
{
    shutdownROS2Node();
}

bool ServiceModel::initializeROS2Node()
{
    try {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        node_ = rclcpp::Node::make_shared("lawer_qt_window_node");
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);

        // 启动执行器线程
        executor_thread_ = std::thread(&ServiceModel::executeInThread, this);

        setupServiceClients();
        node_initialized_ = true;

        emit nodeStatusChanged(true);
        qDebug() << "ROS2 node initialized successfully";
        return true;
    }
    catch (const std::exception& e) {
        qDebug() << "Failed to initialize ROS2 node:" << e.what();
        node_initialized_ = false;
        emit nodeStatusChanged(false);
        return false;
    }
}

void ServiceModel::shutdownROS2Node()
{
    if (node_initialized_) {
        node_initialized_ = false;
        
        if (executor_) {
            executor_->cancel();
        }
        
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }

        service_clients_.clear();
        node_.reset();
        executor_.reset();

        emit nodeStatusChanged(false);
        qDebug() << "ROS2 node shutdown";
    }
}

bool ServiceModel::isNodeInitialized() const
{
    return node_initialized_;
}

void ServiceModel::setupServiceClients()
{
    if (!node_) return;

    // Map Management Services
    service_clients_["/create_free_space"] = node_->create_client<std_srvs::srv::Trigger>("/create_free_space");
    service_clients_["/create_risk_map"] = node_->create_client<std_srvs::srv::Trigger>("/create_risk_map");
    service_clients_["/create_chennal_map"] = node_->create_client<std_srvs::srv::Trigger>("/create_chennal_map");
    service_clients_["/create_zone_cell_decomposition"] = node_->create_client<std_srvs::srv::Trigger>("/create_zone_cell_decomposition");
    service_clients_["/get_record_zone_list_srv"] = node_->create_client<std_srvs::srv::Trigger>("/get_record_zone_list_srv");

    // Path Record Services
    service_clients_["/record_zone_start"] = node_->create_client<std_srvs::srv::Trigger>("/record_zone_start");
    service_clients_["/record_zone_end"] = node_->create_client<std_srvs::srv::Trigger>("/record_zone_end");
    service_clients_["/risk_zone_start"] = node_->create_client<std_srvs::srv::Trigger>("/risk_zone_start");
    service_clients_["/risk_zone_end"] = node_->create_client<std_srvs::srv::Trigger>("/risk_zone_end");
    service_clients_["/save_zone_list"] = node_->create_client<std_srvs::srv::Trigger>("/save_zone_list");
    service_clients_["/risk_zone_save"] = node_->create_client<std_srvs::srv::Trigger>("/risk_zone_save");
    service_clients_["/load_zone_list"] = node_->create_client<std_srvs::srv::Trigger>("/load_zone_list");
    service_clients_["/get_record_zone_info"] = node_->create_client<std_srvs::srv::Trigger>("/get_record_zone_info");
    service_clients_["/chennal_record_start"] = node_->create_client<std_srvs::srv::Trigger>("/chennal_record_start");
    service_clients_["/chennal_record_end"] = node_->create_client<std_srvs::srv::Trigger>("/chennal_record_end");
    service_clients_["/get_chennal_path_list"] = node_->create_client<std_srvs::srv::Trigger>("/get_chennal_path_list");

    // Boustrophedon Coverage Services
    service_clients_["/generate_coverage_path"] = node_->create_client<std_srvs::srv::Trigger>("/generate_coverage_path");
    service_clients_["/waypoint_pub"] = node_->create_client<std_srvs::srv::Trigger>("/waypoint_pub");
    service_clients_["/cencel_nav2"] = node_->create_client<std_srvs::srv::Trigger>("/cencel_nav2");

    qDebug() << "Service clients created:" << service_clients_.size();
}

void ServiceModel::executeInThread()
{
    while (node_initialized_ && rclcpp::ok()) {
        try {
            executor_->spin_once(100ms);
        }
        catch (const std::exception& e) {
            qDebug() << "Executor error:" << e.what();
            break;
        }
    }
}

ServiceModel::ServiceResult ServiceModel::callService(const QString& serviceName)
{
    ServiceResult result;
    result.success = false;

    if (!node_initialized_ || !node_) {
        result.error = "ROS2 node not initialized";
        return result;
    }

    auto client_it = service_clients_.find(serviceName);
    if (client_it == service_clients_.end()) {
        result.error = QString("Service client not found: %1").arg(serviceName);
        return result;
    }

    auto client = client_it->second;
    if (!client->wait_for_service(5s)) {
        result.error = QString("Service not available: %1").arg(serviceName);
        return result;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    // 等待响应
    if (rclcpp::spin_until_future_complete(node_, future, 10s) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        result.success = response->success;
        result.message = QString::fromStdString(response->message);
        
        emit serviceCallCompleted(serviceName, result.success, result.message);
    } else {
        result.error = QString("Service call timeout: %1").arg(serviceName);
    }

    return result;
}

// Map Management Services Implementation
ServiceModel::ServiceResult ServiceModel::createFreeSpace()
{
    return callService("/create_free_space");
}

ServiceModel::ServiceResult ServiceModel::createRiskMap()
{
    return callService("/create_risk_map");
}

ServiceModel::ServiceResult ServiceModel::createChennalMap()
{
    return callService("/create_chennal_map");
}

ServiceModel::ServiceResult ServiceModel::createZoneCellDecomposition()
{
    return callService("/create_zone_cell_decomposition");
}

ServiceModel::ServiceResult ServiceModel::getRecordZoneListSrv()
{
    return callService("/get_record_zone_list_srv");
}

// Path Record Services Implementation
ServiceModel::ServiceResult ServiceModel::recordZoneStart()
{
    return callService("/record_zone_start");
}

ServiceModel::ServiceResult ServiceModel::recordZoneEnd()
{
    return callService("/record_zone_end");
}

ServiceModel::ServiceResult ServiceModel::riskZoneStart()
{
    return callService("/risk_zone_start");
}

ServiceModel::ServiceResult ServiceModel::riskZoneEnd()
{
    return callService("/risk_zone_end");
}

ServiceModel::ServiceResult ServiceModel::saveZoneList()
{
    return callService("/save_zone_list");
}

ServiceModel::ServiceResult ServiceModel::riskZoneSave()
{
    return callService("/risk_zone_save");
}

ServiceModel::ServiceResult ServiceModel::loadZoneList()
{
    return callService("/load_zone_list");
}

ServiceModel::ServiceResult ServiceModel::getRecordZoneInfo()
{
    return callService("/get_record_zone_info");
}

ServiceModel::ServiceResult ServiceModel::chennalRecordStart()
{
    return callService("/chennal_record_start");
}

ServiceModel::ServiceResult ServiceModel::chennalRecordEnd()
{
    return callService("/chennal_record_end");
}

ServiceModel::ServiceResult ServiceModel::getChennalPathList()
{
    return callService("/get_chennal_path_list");
}

// Boustrophedon Coverage Services Implementation
ServiceModel::ServiceResult ServiceModel::generateCoveragePath()
{
    return callService("/generate_coverage_path");
}

ServiceModel::ServiceResult ServiceModel::waypointPub()
{
    return callService("/waypoint_pub");
}

ServiceModel::ServiceResult ServiceModel::cancelNav2()
{
    return callService("/cencel_nav2");
}

void ServiceModel::onServiceResponse()
{
    // 这个槽函数可以用于处理异步响应
    qDebug() << "Service response received";
}
