#include "mainwindow.h"
#include <QApplication>
#include <QSettings>
#include <QDebug>
#include <QAction>
#include <QMenu>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , service_controller_(nullptr)
    , central_widget_(nullptr)
    , main_splitter_(nullptr)
    , right_splitter_(nullptr)
    , control_panel_widget_(nullptr)
    , log_widget_(nullptr)
    , menu_bar_(nullptr)
    , status_bar_(nullptr)
    , connection_status_label_(nullptr)
    , position_label_(nullptr)
    , zoom_label_(nullptr)
    , progress_bar_(nullptr)
    , status_timer_(new QTimer(this))
{
    setupUI();
    connectSignals();
    initializeControllers();
    loadSettings();
    
    // 设置状态更新定时器
    status_timer_->setInterval(STATUS_UPDATE_INTERVAL);
    connect(status_timer_, &QTimer::timeout, this, &MainWindow::onUpdateStatusTimer);
    status_timer_->start();
    
    updateApplicationStatus("正在初始化...");
}

MainWindow::~MainWindow()
{
    saveSettings();
    
    if (service_controller_) {
        service_controller_->shutdown();
    }
}

void MainWindow::setupUI()
{
    setWindowTitle("Lawer Robot Control Panel - Qt6 ROS2 Interface");
    setMinimumSize(800, 600);
    
    // 响应式窗口大小
    QSize screen_size = QGuiApplication::primaryScreen()->availableSize();
    int width = qMin(1200, static_cast<int>(screen_size.width() * 0.8));
    int height = qMin(800, static_cast<int>(screen_size.height() * 0.8));
    resize(width, height);
    
    setupMenuBar();
    setupStatusBar();
    setupCentralWidget();
}

void MainWindow::setupMenuBar()
{
    menu_bar_ = menuBar();
    
    // 文件菜单
    file_menu_ = menu_bar_->addMenu("文件(&F)");
    
    new_action_ = new QAction("新建(&N)", this);
    new_action_->setShortcut(QKeySequence::New);
    file_menu_->addAction(new_action_);
    
    open_action_ = new QAction("打开(&O)", this);
    open_action_->setShortcut(QKeySequence::Open);
    file_menu_->addAction(open_action_);
    
    save_action_ = new QAction("保存(&S)", this);
    save_action_->setShortcut(QKeySequence::Save);
    file_menu_->addAction(save_action_);
    
    file_menu_->addSeparator();
    
    exit_action_ = new QAction("退出(&X)", this);
    exit_action_->setShortcut(QKeySequence::Quit);
    file_menu_->addAction(exit_action_);
    
    // 视图菜单
    view_menu_ = menu_bar_->addMenu("视图(&V)");
    
    // 帮助菜单
    help_menu_ = menu_bar_->addMenu("帮助(&H)");
    
    about_action_ = new QAction("关于(&A)", this);
    help_menu_->addAction(about_action_);
}

void MainWindow::setupStatusBar()
{
    status_bar_ = statusBar();
    
    // 连接状态标签
    connection_status_label_ = new QLabel("未连接");
    connection_status_label_->setStyleSheet("color: red; font-weight: bold;");
    status_bar_->addWidget(connection_status_label_);
    
    status_bar_->addWidget(new QLabel("|"));
    
    // 位置标签
    position_label_ = new QLabel("位置: (0, 0)");
    status_bar_->addWidget(position_label_);
    
    status_bar_->addWidget(new QLabel("|"));
    
    // 缩放标签
    zoom_label_ = new QLabel("缩放: 100%");
    status_bar_->addWidget(zoom_label_);
    
    // 进度条
    progress_bar_ = new QProgressBar();
    progress_bar_->setVisible(false);
    progress_bar_->setMaximumWidth(200);
    status_bar_->addPermanentWidget(progress_bar_);
    
    status_bar_->showMessage("就绪");
}

void MainWindow::setupCentralWidget()
{
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);
    
    // 创建主分割器 (垂直分割 - 上下布局：控制面板 / 日志)
    main_splitter_ = new QSplitter(Qt::Vertical, central_widget_);
    
    // 创建控制面板组件
    control_panel_widget_ = new ControlPanelWidget(this);
    
    // 创建日志组件
    log_widget_ = new LogWidget(this);
    
    // 简化布局：只有控制面板和日志
    main_splitter_->addWidget(control_panel_widget_);
    main_splitter_->addWidget(log_widget_);
    
    // 设置组件的最小尺寸
    control_panel_widget_->setMinimumHeight(400);
    log_widget_->setMinimumHeight(200);
    log_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    
    // 设置初始分割器比例：控制面板60% / 日志40%
    main_splitter_->setSizes({500, 300});
    
    // 调试信息
    qDebug() << "MainWindow: 简化布局 - 只有控制面板和日志";
    qDebug() << "MainWindow: Main splitter widget count:" << main_splitter_->count();
    qDebug() << "MainWindow: LogWidget minimum height:" << log_widget_->minimumHeight();
    
    // 响应式设置
    setupResponsiveSplitters();
    
    // 设置中央组件布局
    QVBoxLayout* layout = new QVBoxLayout(central_widget_);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->addWidget(main_splitter_);
}

void MainWindow::connectSignals()
{
    // 菜单动作信号
    connect(new_action_, &QAction::triggered, this, &MainWindow::onNewSession);
    connect(open_action_, &QAction::triggered, this, &MainWindow::onOpenSession);
    connect(save_action_, &QAction::triggered, this, &MainWindow::onSaveSession);
    connect(exit_action_, &QAction::triggered, this, &MainWindow::onExit);
    connect(about_action_, &QAction::triggered, this, &MainWindow::onAbout);
    
    // 连接日志功能
    connectLogSignals();
}

void MainWindow::connectLogSignals()
{
    if (!log_widget_) return;
    
    // 连接服务控制器的日志信号
    if (service_controller_) {
        connect(service_controller_, &ServiceController::serviceCallStarted,
                this, [this](const QString& service_name) {
                    log_widget_->addServiceCall(service_name, "std_srvs/srv/Trigger", true);
                });
        
        connect(service_controller_, &ServiceController::serviceCallCompleted,
                this, [this](const QString& service_name, bool success, const QString& message) {
                    LogWidget::LogType type = success ? LogWidget::SUCCESS : LogWidget::ERROR;
                    QString log_message = QString("服务 %1 %2: %3")
                                         .arg(service_name)
                                         .arg(success ? "成功" : "失败")
                                         .arg(message);
                    log_widget_->addLog(type, log_message);
                });
        
        connect(service_controller_, &ServiceController::serviceCallFailed,
                this, [this](const QString& service_name, const QString& error) {
                    QString log_message = QString("服务调用失败 %1: %2").arg(service_name).arg(error);
                    log_widget_->addLog(LogWidget::ERROR, log_message);
                });
    }
    
    // 记录初始化日志
    log_widget_->addLog(LogWidget::INFO, "ROS2日志监控已启动 - 简化版本（仅服务调用）");
}

void MainWindow::initializeControllers()
{
    // 只创建服务控制器（移除地图和路径控制器）
    service_controller_ = new ServiceController(this);
    
    // *** 重要：先连接信号，再初始化 ***
    qDebug() << "MainWindow: Connecting ServiceController signals";
    connect(service_controller_, &ServiceController::initializationCompleted,
            this, &MainWindow::onServiceControllerInitialized);
    connect(service_controller_, &ServiceController::serviceCallStarted,
            this, &MainWindow::onServiceCallStarted);
    connect(service_controller_, &ServiceController::serviceCallCompleted,
            this, &MainWindow::onServiceCallCompleted);
    connect(service_controller_, &ServiceController::serviceCallFailed,
            this, &MainWindow::onServiceCallFailed);
    
    // 开始初始化进度显示
    showInitializationProgress();
    
    // *** 先初始化 ServiceController ***
    qDebug() << "MainWindow: Starting ServiceController initialization";
    service_controller_->initialize();
    
    // *** 然后设置到控制面板 ***
    if (control_panel_widget_) {
        qDebug() << "MainWindow: Setting ServiceController to ControlPanelWidget";
        control_panel_widget_->setServiceController(service_controller_);
    }
    
    // 添加延迟检查，以防信号丢失
    QTimer::singleShot(500, this, [this]() {
        if (service_controller_ && service_controller_->isInitialized()) {
            qDebug() << "Manual check: ServiceController is initialized";
            // 检查连接状态标签是否还显示"未连接"
            if (connection_status_label_->text() == "未连接") {
                qDebug() << "UI was not updated, manually triggering onServiceControllerInitialized";
                onServiceControllerInitialized(true);
            } else {
                qDebug() << "UI already updated, connection status:" << connection_status_label_->text();
            }
        } else {
            qDebug() << "Manual check: ServiceController not yet initialized";
        }
    });
}

void MainWindow::showInitializationProgress()
{
    progress_bar_->setVisible(true);
    progress_bar_->setRange(0, 0);  // 无限进度条
    updateApplicationStatus("正在初始化ROS2服务...");
}

void MainWindow::onServiceControllerInitialized(bool success)
{
    qDebug() << "MainWindow::onServiceControllerInitialized:" << success;
    
    if (success) {
        connection_status_label_->setText("已连接");
        connection_status_label_->setStyleSheet("color: green; font-weight: bold;");
        updateApplicationStatus("服务控制器初始化成功", 3000);
        
        // 强制更新控制面板按钮状态
        if (control_panel_widget_) {
            qDebug() << "Updating button states after ServiceController initialization";
            // 使用 QTimer::singleShot 确保在事件循环中更新
            QTimer::singleShot(0, [this]() {
                if (control_panel_widget_) {
                    control_panel_widget_->updateButtonStates();
                }
            });
        }
    } else {
        connection_status_label_->setText("连接失败");
        connection_status_label_->setStyleSheet("color: red; font-weight: bold;");
        updateApplicationStatus("服务控制器初始化失败", 5000);
    }
    
    progress_bar_->setVisible(false);
}

void MainWindow::onServiceCallStarted(const QString& serviceName)
{
    progress_bar_->setVisible(true);
    progress_bar_->setRange(0, 0);
    updateApplicationStatus(QString("正在执行: %1").arg(serviceName));
}

void MainWindow::onServiceCallCompleted(const QString& serviceName, bool success, const QString& message)
{
    Q_UNUSED(success)
    progress_bar_->setVisible(false);
    
    QString status_msg = QString("服务完成: %1").arg(serviceName);
    if (!message.isEmpty()) {
        status_msg += QString(" - %1").arg(message);
    }
    
    updateApplicationStatus(status_msg, 3000);
}

void MainWindow::onServiceCallFailed(const QString& serviceName, const QString& error)
{
    progress_bar_->setVisible(false);
    updateApplicationStatus(QString("服务失败: %1 - %2").arg(serviceName).arg(error), 5000);
}

void MainWindow::setupResponsiveSplitters()
{
    // 简化的响应式布局：只有控制面板和日志
    int window_height = height();
    
    // 设置日志面板的高度 (固定比例，但确保最小高度)
    int log_height = qMax(200, static_cast<int>(window_height * 0.35));    // 日志占35%高度，最小200px
    int control_height = window_height - log_height - 100;                 // 控制面板占剩余高度
    
    // 设置控制面板和日志的最小高度
    control_panel_widget_->setMinimumHeight(300);
    log_widget_->setMinimumHeight(200);
    
    // 设置分割器比例：控制面板 / 日志
    main_splitter_->setSizes({control_height, log_height});
    
    qDebug() << "MainWindow: 响应式布局更新";
    qDebug() << "  - 窗口高度:" << window_height;
    qDebug() << "  - 控制面板高度:" << control_height;
    qDebug() << "  - 日志面板高度:" << log_height;
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    
    // 窗口大小改变时重新调整分割器
    if (main_splitter_) {
        setupResponsiveSplitters();
    }
}

// 菜单事件处理
void MainWindow::onNewSession()
{
    updateApplicationStatus("新建会话", 2000);
}

void MainWindow::onOpenSession()
{
    updateApplicationStatus("打开会话", 2000);
}

void MainWindow::onSaveSession()
{
    updateApplicationStatus("保存会话", 2000);
}

void MainWindow::onExit()
{
    close();
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, "关于",
                      "Lawer Robot Control Panel\n"
                      "版本: 1.0\n"
                      "基于Qt6和ROS2的机器人控制界面");
}

void MainWindow::onUpdateStatusTimer()
{
    updateConnectionStatus();
}

void MainWindow::updateConnectionStatus()
{
    if (service_controller_ && service_controller_->isInitialized()) {
        connection_status_label_->setText("已连接");
        connection_status_label_->setStyleSheet("color: green; font-weight: bold;");
    } else {
        connection_status_label_->setText("未连接");
        connection_status_label_->setStyleSheet("color: red; font-weight: bold;");
    }
}

void MainWindow::updateApplicationStatus(const QString& message, int timeout)
{
    status_bar_->showMessage(message, timeout);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (confirmExit()) {
        saveSettings();
        event->accept();
    } else {
        event->ignore();
    }
}

void MainWindow::saveSettings()
{
    QSettings settings;
    settings.beginGroup("MainWindow");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    
    // 保存分割器状态
    if (main_splitter_) {
        settings.setValue("mainSplitterSizes", main_splitter_->saveState());
    }
    
    settings.endGroup();
}

void MainWindow::loadSettings()
{
    QSettings settings;
    settings.beginGroup("MainWindow");
    
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    
    // 恢复分割器状态
    if (main_splitter_) {
        main_splitter_->restoreState(settings.value("mainSplitterSizes").toByteArray());
    }
    
    settings.endGroup();
}

bool MainWindow::confirmExit()
{
    // 简化版本，直接退出
    return true;
}
