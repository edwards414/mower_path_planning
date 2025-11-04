#include "ServiceController.h"
#include <QDebug>
#include <QApplication>
#include <functional>

ServiceController::ServiceController(QObject *parent)
    : QObject(parent)
    , service_model_(nullptr)
    , initialization_timer_(nullptr)
    , is_initialized_(false)
{
    service_model_ = new ServiceModel(this);
    initialization_timer_ = new QTimer(this);
    initialization_timer_->setSingleShot(true);
    initialization_timer_->setInterval(10000);  // 10秒超时

    // 连接信号
    connect(service_model_, &ServiceModel::serviceCallCompleted,
            this, &ServiceController::onServiceCallCompleted);
    connect(service_model_, &ServiceModel::nodeStatusChanged,
            this, &ServiceController::onNodeStatusChanged);
    connect(initialization_timer_, &QTimer::timeout,
            this, &ServiceController::onInitializationTimeout);
}

ServiceController::~ServiceController()
{
    shutdown();
}

bool ServiceController::initialize()
{
    if (is_initialized_) {
        return true;
    }

    qDebug() << "Initializing ServiceController...";
    
    initialization_timer_->start();
    bool success = service_model_->initializeROS2Node();
    
    if (!success) {
        initialization_timer_->stop();
        last_error_ = "Failed to initialize ROS2 node";
        emit initializationCompleted(false);
        return false;
    }

    return true;  // 实际的初始化完成会通过信号通知
}

void ServiceController::shutdown()
{
    if (is_initialized_) {
        qDebug() << "Shutting down ServiceController...";
        
        if (initialization_timer_->isActive()) {
            initialization_timer_->stop();
        }
        
        service_model_->shutdownROS2Node();
        is_initialized_ = false;
        
        emit shutdownCompleted();
    }
}

void ServiceController::onNodeStatusChanged(bool initialized)
{
    qDebug() << "ServiceController::onNodeStatusChanged called with:" << initialized;
    initialization_timer_->stop();
    is_initialized_ = initialized;
    
    if (initialized) {
        qDebug() << "ServiceController initialized successfully";
        last_error_.clear();
    } else {
        last_error_ = "ROS2 node initialization failed";
        qDebug() << last_error_;
    }
    
    qDebug() << "ServiceController: Emitting initializationCompleted(" << initialized << ") signal";
    emit initializationCompleted(initialized);
    qDebug() << "ServiceController: initializationCompleted signal emitted";
}

void ServiceController::onInitializationTimeout()
{
    is_initialized_ = false;
    last_error_ = "Initialization timeout";
    qDebug() << last_error_;
    emit initializationCompleted(false);
}

void ServiceController::onServiceCallCompleted(const QString& serviceName, bool success, const QString& message)
{
    if (success) {
        emit serviceCallCompleted(serviceName, success, message);
        qDebug() << "Service call completed:" << serviceName << "Message:" << message;
    } else {
        emit serviceCallFailed(serviceName, message);
        qDebug() << "Service call failed:" << serviceName << "Error:" << message;
    }
}

void ServiceController::executeServiceCall(const QString& serviceName, 
                                         std::function<ServiceModel::ServiceResult()> serviceCall)
{
    if (!is_initialized_) {
        emit serviceCallFailed(serviceName, "ServiceController not initialized");
        return;
    }

    // 创建服务名称映射，显示实际的ROS2服务路径
    QString ros2_service_path = QString("/%1").arg(serviceName);
    QString display_name = QString("%1 -> %2").arg(serviceName).arg(ros2_service_path);
    
    emit serviceCallStarted(display_name);
    qDebug() << "Starting service call:" << serviceName << "-> ROS2 service:" << ros2_service_path;

    // 在单独的线程中执行服务调用以避免阻塞UI
    QTimer::singleShot(0, [this, serviceName, serviceCall]() {
        try {
            auto result = serviceCall();
            
            if (result.success) {
                emit serviceCallCompleted(serviceName, true, result.message);
            } else {
                QString error = result.error.isEmpty() ? result.message : result.error;
                emit serviceCallFailed(serviceName, error);
            }
        }
        catch (const std::exception& e) {
            emit serviceCallFailed(serviceName, QString("Exception: %1").arg(e.what()));
        }
    });
}

bool ServiceController::isInitialized() const
{
    return is_initialized_;
}

QString ServiceController::getLastError() const
{
    return last_error_;
}

// Map Management Services Implementation
void ServiceController::createFreeSpace()
{
    executeServiceCall("create_free_space", [this]() {
        return service_model_->createFreeSpace();
    });
}

void ServiceController::createRiskMap()
{
    executeServiceCall("create_risk_map", [this]() {
        return service_model_->createRiskMap();
    });
}

void ServiceController::createChennalMap()
{
    executeServiceCall("create_chennal_map", [this]() {
        return service_model_->createChennalMap();
    });
}

void ServiceController::createZoneCellDecomposition()
{
    executeServiceCall("create_zone_cell_decomposition", [this]() {
        return service_model_->createZoneCellDecomposition();
    });
}

void ServiceController::getRecordZoneListSrv()
{
    executeServiceCall("get_record_zone_list_srv", [this]() {
        return service_model_->getRecordZoneListSrv();
    });
}

// Path Record Services Implementation
void ServiceController::recordZoneStart()
{
    executeServiceCall("record_zone_start", [this]() {
        return service_model_->recordZoneStart();
    });
}

void ServiceController::recordZoneEnd()
{
    executeServiceCall("record_zone_end", [this]() {
        return service_model_->recordZoneEnd();
    });
}

void ServiceController::riskZoneStart()
{
    executeServiceCall("risk_zone_start", [this]() {
        return service_model_->riskZoneStart();
    });
}

void ServiceController::riskZoneEnd()
{
    executeServiceCall("risk_zone_end", [this]() {
        return service_model_->riskZoneEnd();
    });
}

void ServiceController::saveZoneList()
{
    executeServiceCall("save_zone_list", [this]() {
        return service_model_->saveZoneList();
    });
}

void ServiceController::riskZoneSave()
{
    executeServiceCall("risk_zone_save", [this]() {
        return service_model_->riskZoneSave();
    });
}

void ServiceController::loadZoneList()
{
    executeServiceCall("load_zone_list", [this]() {
        return service_model_->loadZoneList();
    });
}

void ServiceController::getRecordZoneInfo()
{
    executeServiceCall("get_record_zone_info", [this]() {
        return service_model_->getRecordZoneInfo();
    });
}

void ServiceController::chennalRecordStart()
{
    executeServiceCall("chennal_record_start", [this]() {
        return service_model_->chennalRecordStart();
    });
}

void ServiceController::chennalRecordEnd()
{
    executeServiceCall("chennal_record_end", [this]() {
        return service_model_->chennalRecordEnd();
    });
}

void ServiceController::getChennalPathList()
{
    executeServiceCall("get_chennal_path_list", [this]() {
        return service_model_->getChennalPathList();
    });
}

// Boustrophedon Coverage Services Implementation
void ServiceController::generateCoveragePath()
{
    executeServiceCall("generate_coverage_path", [this]() {
        return service_model_->generateCoveragePath();
    });
}

void ServiceController::waypointPub()
{
    executeServiceCall("waypoint_pub", [this]() {
        return service_model_->waypointPub();
    });
}

void ServiceController::cancelNav2()
{
    executeServiceCall("cancel_nav2", [this]() {
        return service_model_->cancelNav2();
    });
}
