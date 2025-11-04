#ifndef SERVICECONTROLLER_H
#define SERVICECONTROLLER_H

#include <QObject>
#include <QTimer>
#include "../models/ServiceModel.h"

class ServiceController : public QObject
{
    Q_OBJECT

public:
    explicit ServiceController(QObject *parent = nullptr);
    ~ServiceController();

    // 初始化和清理
    bool initialize();
    void shutdown();

    // 获取服务模型
    ServiceModel* getServiceModel() const { return service_model_; }

    // Map Management Services
    Q_INVOKABLE void createFreeSpace();
    Q_INVOKABLE void createRiskMap();
    Q_INVOKABLE void createChennalMap();
    Q_INVOKABLE void createZoneCellDecomposition();
    Q_INVOKABLE void getRecordZoneListSrv();

    // Path Record Services
    Q_INVOKABLE void recordZoneStart();
    Q_INVOKABLE void recordZoneEnd();
    Q_INVOKABLE void riskZoneStart();
    Q_INVOKABLE void riskZoneEnd();
    Q_INVOKABLE void saveZoneList();
    Q_INVOKABLE void riskZoneSave();
    Q_INVOKABLE void loadZoneList();
    Q_INVOKABLE void getRecordZoneInfo();
    Q_INVOKABLE void chennalRecordStart();
    Q_INVOKABLE void chennalRecordEnd();
    Q_INVOKABLE void getChennalPathList();

    // Boustrophedon Coverage Services
    Q_INVOKABLE void generateCoveragePath();
    Q_INVOKABLE void waypointPub();
    Q_INVOKABLE void cancelNav2();

    // 状态查询
    bool isInitialized() const;
    QString getLastError() const;

signals:
    void serviceCallStarted(const QString& serviceName);
    void serviceCallCompleted(const QString& serviceName, bool success, const QString& message);
    void serviceCallFailed(const QString& serviceName, const QString& error);
    void initializationCompleted(bool success);
    void shutdownCompleted();

private slots:
    void onServiceCallCompleted(const QString& serviceName, bool success, const QString& message);
    void onNodeStatusChanged(bool initialized);
    void onInitializationTimeout();

private:
    ServiceModel* service_model_;
    QTimer* initialization_timer_;
    bool is_initialized_;
    QString last_error_;

    // 私有方法
    void executeServiceCall(const QString& serviceName, 
                           std::function<ServiceModel::ServiceResult()> serviceCall);
};

#endif // SERVICECONTROLLER_H
