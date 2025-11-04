#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QMenuBar>
#include <QStatusBar>
#include <QLabel>
#include <QProgressBar>
#include <QTimer>
#include <QCloseEvent>
#include <QResizeEvent>
#include <QMessageBox>
#include <QGuiApplication>
#include <QScreen>

// 控制器
#include "controllers/ServiceController.h"

// 组件
#include "widgets/ControlPanelWidget.h"
#include "widgets/LogWidget.h"

#include "controllers/MapController.h"
#include "controllers/PathController.h"

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    // 菜单动作
    void onNewSession();
    void onOpenSession();
    void onSaveSession();
    void onExit();
    void onAbout();
    
    // 控制器事件
    void onServiceControllerInitialized(bool success);
    
    // 服务事件
    void onServiceCallStarted(const QString& serviceName);
    void onServiceCallCompleted(const QString& serviceName, bool success, const QString& message);
    void onServiceCallFailed(const QString& serviceName, const QString& error);
    
    // 状态更新
    void onUpdateStatusTimer();

private:
    // 初始化方法
    void setupUI();
    void setupMenuBar();
    void setupStatusBar();
    void setupCentralWidget();
    void connectSignals();
    void connectLogSignals();
    void initializeControllers();
    
    // 状态管理
    void updateConnectionStatus();
    void updateApplicationStatus(const QString& message, int timeout = 0);
    void showInitializationProgress();
    void checkInitializationComplete();
    
    // 响应式设计
    void setupResponsiveSplitters();
    
    // 会话管理
    void saveSettings();
    void loadSettings();
    bool confirmExit();

private:
    // 控制器
    ServiceController* service_controller_;
    
    // 主要组件
    QWidget* central_widget_;
    QSplitter* main_splitter_;
    QSplitter* right_splitter_;
    
    ControlPanelWidget* control_panel_widget_;
    LogWidget* log_widget_;
    
    // 菜单和工具栏
    QMenuBar* menu_bar_;
    QMenu* file_menu_;
    QMenu* view_menu_;
    QMenu* help_menu_;
    
    QAction* new_action_;
    QAction* open_action_;
    QAction* save_action_;
    QAction* exit_action_;
    QAction* about_action_;
    
    // 状态栏
    QStatusBar* status_bar_;
    QLabel* connection_status_label_;
    QLabel* position_label_;
    QLabel* zoom_label_;
    QProgressBar* progress_bar_;
    
    // 状态管理
    QTimer* status_timer_;
    bool controllers_initialized_;
    int initialization_count_;
    
    // 常量
    static constexpr int STATUS_UPDATE_INTERVAL = 1000;  // 1秒
};

#endif // MAINWINDOW_H
