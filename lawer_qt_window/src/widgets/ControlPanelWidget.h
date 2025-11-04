#ifndef CONTROLPANELWIDGET_H
#define CONTROLPANELWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTextEdit>
#include <QProgressBar>
#include <QTimer>
#include <QScrollArea>
#include <QFrame>
#include <QResizeEvent>
#include <QVector>
#include "../controllers/ServiceController.h"

class ControlPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanelWidget(QWidget *parent = nullptr);
    ~ControlPanelWidget();

    // 设置服务控制器
    void setServiceController(ServiceController* controller);

    // 状态管理
    void setEnabled(bool enabled);
    void showMessage(const QString& message, bool is_error = false);
    void clearMessages();
    void updateButtonStates();

signals:
    void serviceRequested(const QString& serviceName);
    void statusChanged(const QString& status);

private slots:
    // Map Management Services
    void onCreateFreeSpaceClicked();
    void onCreateRiskMapClicked();
    void onCreateChennalMapClicked();
    void onCreateZoneCellDecompositionClicked();
    void onGetRecordZoneListSrvClicked();

    // Path Record Services
    void onRecordZoneStartClicked();
    void onRecordZoneEndClicked();
    void onRiskZoneStartClicked();
    void onRiskZoneEndClicked();
    void onSaveZoneListClicked();
    void onRiskZoneSaveClicked();
    void onLoadZoneListClicked();
    void onGetRecordZoneInfoClicked();
    void onChennalRecordStartClicked();
    void onChennalRecordEndClicked();
    void onGetChennalPathListClicked();

    // Boustrophedon Coverage Services
    void onGenerateCoveragePathClicked();
    void onWaypointPubClicked();
    void onCancelNav2Clicked();

    // Service Controller 事件
    void onServiceCallStarted(const QString& serviceName);
    void onServiceCallCompleted(const QString& serviceName, bool success, const QString& message);
    void onServiceCallFailed(const QString& serviceName, const QString& error);

    // UI 更新
    void onProgressTimeout();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    ServiceController* service_controller_;

    // UI 组件
    QVBoxLayout* main_layout_;
    
    // Map Management Group
    QGroupBox* map_management_group_;
    QPushButton* create_free_space_btn_;
    QPushButton* create_risk_map_btn_;
    QPushButton* create_chennal_map_btn_;
    QPushButton* create_bcd_btn_;
    QPushButton* get_zone_list_btn_;

    // Path Record Group
    QGroupBox* path_record_group_;
    QPushButton* record_zone_start_btn_;
    QPushButton* record_zone_end_btn_;
    QPushButton* risk_zone_start_btn_;
    QPushButton* risk_zone_end_btn_;
    QPushButton* save_zone_list_btn_;
    QPushButton* risk_zone_save_btn_;
    QPushButton* load_zone_list_btn_;
    QPushButton* get_record_info_btn_;
    QPushButton* chennal_record_start_btn_;
    QPushButton* chennal_record_end_btn_;
    QPushButton* get_chennal_list_btn_;

    // Coverage Planning Group
    QGroupBox* coverage_planning_group_;
    QPushButton* generate_coverage_path_btn_;
    QPushButton* waypoint_pub_btn_;
    QPushButton* cancel_nav2_btn_;

    // Status Group
    QGroupBox* status_group_;
    QLabel* status_label_;
    QProgressBar* progress_bar_;
    QTextEdit* message_display_;

    // 私有方法
    void setupUI();
    void setupMapManagementGroup();
    void setupPathRecordGroup();
    void setupCoveragePlanningGroup();
    void setupStatusGroup();
    void connectSignals();

    // 按钮样式
    void setupButtonStyle(QPushButton* button, const QString& color = "#4CAF50");
    void setButtonBusy(QPushButton* button, bool busy);
    QString adjustColor(const QString& color, int adjustment);
    QString getContrastColor(const QString& color);
    
    // 响应式设计
    void createResponsiveButtonGrid(QVBoxLayout* parent_layout, 
                                   const QVector<QVector<QPushButton*>>& button_rows);
    void adjustButtonTextForWidth(int width);

    // 状态管理
    QTimer* progress_timer_;
    QString current_service_;
    bool is_service_running_;
};

#endif // CONTROLPANELWIDGET_H
