#ifndef SERVICEMODEL_H
#define SERVICEMODEL_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ServiceModel : public QObject
{
    Q_OBJECT

public:
    explicit ServiceModel(QObject *parent = nullptr);
    ~ServiceModel();

    // 服务调用结果结构
    struct ServiceResult {
        bool success;
        QString message;
        QString error;
    };

    // Map Management Services
    Q_INVOKABLE ServiceResult createFreeSpace();
    Q_INVOKABLE ServiceResult createRiskMap();
    Q_INVOKABLE ServiceResult createChennalMap();
    Q_INVOKABLE ServiceResult createZoneCellDecomposition();
    Q_INVOKABLE ServiceResult getRecordZoneListSrv();

    // Path Record Services
    Q_INVOKABLE ServiceResult recordZoneStart();
    Q_INVOKABLE ServiceResult recordZoneEnd();
    Q_INVOKABLE ServiceResult riskZoneStart();
    Q_INVOKABLE ServiceResult riskZoneEnd();
    Q_INVOKABLE ServiceResult saveZoneList();
    Q_INVOKABLE ServiceResult riskZoneSave();
    Q_INVOKABLE ServiceResult loadZoneList();
    Q_INVOKABLE ServiceResult getRecordZoneInfo();
    Q_INVOKABLE ServiceResult chennalRecordStart();
    Q_INVOKABLE ServiceResult chennalRecordEnd();
    Q_INVOKABLE ServiceResult getChennalPathList();

    // Boustrophedon Coverage Services
    Q_INVOKABLE ServiceResult generateCoveragePath();
    Q_INVOKABLE ServiceResult waypointPub();
    Q_INVOKABLE ServiceResult cancelNav2();

    // ROS2 节点管理
    bool initializeROS2Node();
    void shutdownROS2Node();
    bool isNodeInitialized() const;
    
    // 获取ROS2节点 (用于其他控制器)
    rclcpp::Node::SharedPtr getNode() const { return node_; }

signals:
    void serviceCallCompleted(const QString& serviceName, bool success, const QString& message);
    void nodeStatusChanged(bool initialized);

private slots:
    void onServiceResponse();

private:
    // ROS2 相关
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    bool node_initialized_;

    // 服务客户端
    std::map<QString, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> service_clients_;

    // 私有方法
    ServiceResult callService(const QString& serviceName);
    void setupServiceClients();
    void executeInThread();
};

#endif // SERVICEMODEL_H
