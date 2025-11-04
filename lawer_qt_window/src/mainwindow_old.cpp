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
    , menu_bar_(nullptr)
    , status_bar_(nullptr)
    , connection_status_label_(nullptr)
    , position_label_(nullptr)
    , zoom_label_(nullptr)
    , progress_bar_(nullptr)
    , status_timer_(new QTimer(this))
    , controllers_initialized_(false)
    , initialization_count_(0)
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
    // 地图和路径控制器已移除
}

void MainWindow::setupUI()
{
    setWindowTitle("Lawer Robot Control Panel - Qt6 ROS2 Interface");
    setMinimumSize(800, 600);  // 降低最小尺寸以支持小屏幕
    
    // 响应式窗口大小
    QSize screen_size = QGuiApplication::primaryScreen()->availableSize();
    int width = qMin(1600, static_cast<int>(screen_size.width() * 0.9));
    int height = qMin(1000, static_cast<int>(screen_size.height() * 0.9));
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
    
    new_action_ = new QAction("新建会话(&N)", this);
    new_action_->setShortcut(QKeySequence::New);
    new_action_->setStatusTip("创建新的控制会话");
    file_menu_->addAction(new_action_);
    
    open_action_ = new QAction("打开会话(&O)", this);
    open_action_->setShortcut(QKeySequence::Open);
    open_action_->setStatusTip("打开保存的控制会话");
    file_menu_->addAction(open_action_);
    
    save_action_ = new QAction("保存会话(&S)", this);
    save_action_->setShortcut(QKeySequence::Save);
    save_action_->setStatusTip("保存当前控制会话");
    file_menu_->addAction(save_action_);
    
    file_menu_->addSeparator();
    
    exit_action_ = new QAction("退出(&X)", this);
    exit_action_->setShortcut(QKeySequence::Quit);
    exit_action_->setStatusTip("退出应用程序");
    file_menu_->addAction(exit_action_);
    
    // 视图菜单
    view_menu_ = menu_bar_->addMenu("视图(&V)");
    
    // 帮助菜单
    help_menu_ = menu_bar_->addMenu("帮助(&H)");
    
    about_action_ = new QAction("关于(&A)", this);
    about_action_->setStatusTip("显示应用程序信息");
    help_menu_->addAction(about_action_);
}

void MainWindow::setupStatusBar()
{
    status_bar_ = statusBar();
    
    // 连接状态标签
    connection_status_label_ = new QLabel("未连接", this);
    connection_status_label_->setStyleSheet("QLabel { color: red; }");
    connection_status_label_->setMinimumWidth(80);
    status_bar_->addPermanentWidget(connection_status_label_);
    
    // 鼠标位置标签
    position_label_ = new QLabel("位置: (0.00, 0.00)", this);
    position_label_->setMinimumWidth(150);
    status_bar_->addPermanentWidget(position_label_);
    
    // 缩放级别标签
    zoom_label_ = new QLabel("缩放: 100%", this);
    zoom_label_->setMinimumWidth(80);
    status_bar_->addPermanentWidget(zoom_label_);
    
    // 进度条
    progress_bar_ = new QProgressBar(this);
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
    
    // 地图组件已移除，不需要连接地图信号
    
    // 路径可视化组件已移除，不需要连接相关信号
    
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
    
    // 地图和路径控制器已移除，不再连接相关日志信号
    
    // 记录初始化日志
    log_widget_->addLog(LogWidget::INFO, "ROS2日志监控已启动 - 简化版本（仅服务调用）");
}

void MainWindow::initializeControllers()
{
    // 只创建服务控制器（移除地图和路径控制器）
    service_controller_ = new ServiceController(this);
    
    // 连接服务控制器信号
    connect(service_controller_, &ServiceController::initializationCompleted,
            this, &MainWindow::onServiceControllerInitialized);
    connect(service_controller_, &ServiceController::serviceCallStarted,
            this, &MainWindow::onServiceCallStarted);
    connect(service_controller_, &ServiceController::serviceCallCompleted,
            this, &MainWindow::onServiceCallCompleted);
    connect(service_controller_, &ServiceController::serviceCallFailed,
            this, &MainWindow::onServiceCallFailed);
    
    // 设置控制面板的服务控制器
    if (control_panel_widget_) {
        control_panel_widget_->setServiceController(service_controller_);
    }
    
    // 开始初始化
    showInitializationProgress();
    service_controller_->initialize();
}

void MainWindow::showInitializationProgress()
{
    progress_bar_->setVisible(true);
    progress_bar_->setRange(0, 0);  // 无限进度条
    updateApplicationStatus("正在初始化控制器...");
}

void MainWindow::onServiceControllerInitialized(bool success)
{
    initialization_count_++;
    
    if (success) {
        qDebug() << "Service controller initialized successfully";
        
        // 初始化地图和路径控制器 (需要ROS2节点)
        auto node = service_controller_->getServiceModel()->getNode();
        if (node) {
            map_controller_->initialize(node);
            path_controller_->initialize(node);
        }
    } else {
        qDebug() << "Service controller initialization failed";
        updateApplicationStatus("服务控制器初始化失败", 5000);
        progress_bar_->setVisible(false);
    }
    
    updateConnectionStatus();
}

void MainWindow::onMapControllerInitialized(bool success)
{
    initialization_count_++;
    
    if (success) {
        qDebug() << "Map controller initialized successfully";
    } else {
        qDebug() << "Map controller initialization failed";
    }
    
    checkInitializationComplete();
}

void MainWindow::onPathControllerInitialized(bool success)
{
    initialization_count_++;
    
    if (success) {
        qDebug() << "Path controller initialized successfully";
    } else {
        qDebug() << "Path controller initialization failed";
    }
    
    checkInitializationComplete();
}

void MainWindow::checkInitializationComplete()
{
    if (initialization_count_ >= 3) {  // 所有三个控制器都已初始化
        bool all_success = service_controller_->isInitialized() &&
                          map_controller_->isInitialized() &&
                          path_controller_->isInitialized();
        
        controllers_initialized_ = all_success;
        progress_bar_->setVisible(false);
        
        if (all_success) {
            updateApplicationStatus("初始化完成，系统就绪", 3000);
        } else {
            updateApplicationStatus("部分控制器初始化失败", 5000);
        }
        
        updateConnectionStatus();
    }
}

void MainWindow::updateConnectionStatus()
{
    if (controllers_initialized_) {
        connection_status_label_->setText("已连接");
        connection_status_label_->setStyleSheet("QLabel { color: green; }");
    } else {
        connection_status_label_->setText("未连接");
        connection_status_label_->setStyleSheet("QLabel { color: red; }");
    }
}

void MainWindow::updateApplicationStatus(const QString& message, int timeout)
{
    status_bar_->showMessage(message, timeout);
}

// 菜单动作槽函数
void MainWindow::onNewSession()
{
    // 实现新建会话逻辑
    updateApplicationStatus("新建会话", 2000);
}

void MainWindow::onOpenSession()
{
    // 实现打开会话逻辑
    updateApplicationStatus("打开会话", 2000);
}

void MainWindow::onSaveSession()
{
    // 实现保存会话逻辑
    saveSettings();
    updateApplicationStatus("会话已保存", 2000);
}

void MainWindow::onExit()
{
    close();
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, "关于 Lawer Robot Control Panel",
                      "Lawer Robot Control Panel v1.0\n\n"
                      "基于Qt6和ROS2的机器人控制界面\n"
                      "支持地图管理、路径规划和覆盖路径生成\n\n"
                      "开发者: fxrbindi\n"
                      "邮箱: edwards940428@gmail.com");
}

// 服务事件槽函数
void MainWindow::onServiceCallStarted(const QString& serviceName)
{
    progress_bar_->setVisible(true);
    progress_bar_->setRange(0, 0);
    updateApplicationStatus(QString("正在执行: %1").arg(serviceName));
}

void MainWindow::onServiceCallCompleted(const QString& serviceName, bool success, const QString& message)
{
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

// 地图事件槽函数
void MainWindow::onMapClicked(const QPointF& map_point)
{
    updateApplicationStatus(QString("地图点击: (%.2f, %.2f)").arg(map_point.x()).arg(map_point.y()), 2000);
}

void MainWindow::onMapDoubleClicked(const QPointF& map_point)
{
    updateApplicationStatus(QString("地图双击: (%.2f, %.2f)").arg(map_point.x()).arg(map_point.y()), 2000);
}

void MainWindow::onViewChanged(const QPointF& center, float zoom)
{
    zoom_label_->setText(QString("缩放: %1%").arg(static_cast<int>(zoom * 100)));
}

void MainWindow::onMousePositionChanged(const QPointF& map_point)
{
    position_label_->setText(QString("位置: (%.2f, %.2f)").arg(map_point.x()).arg(map_point.y()));
}

// 可视化控制事件槽函数
void MainWindow::onPathVisibilityChanged(const QString& path_name, bool visible)
{
    if (map_widget_) {
        map_widget_->setPathLayerEnabled(path_name, visible);
    }
}

void MainWindow::onMapVisibilityChanged(const QString& map_name, bool visible)
{
    if (map_widget_) {
        map_widget_->setMapLayerEnabled(map_name, visible);
    }
}

void MainWindow::onViewControlRequested(const QString& action)
{
    if (!map_widget_) return;
    
    if (action == "fit_to_content") {
        map_widget_->fitToContent();
        updateApplicationStatus("视图已适应内容", 2000);
    } else if (action == "reset_view") {
        map_widget_->resetView();
        updateApplicationStatus("视图已重置", 2000);
    } else if (action == "clear_paths") {
        if (path_controller_) {
            path_controller_->clearAllPaths();
            updateApplicationStatus("路径已清除", 2000);
        }
    } else if (action == "grid") {
        QCheckBox* checkbox = qobject_cast<QCheckBox*>(sender());
        if (checkbox) {
            map_widget_->setGridEnabled(checkbox->isChecked());
        }
    } else if (action == "robot") {
        QCheckBox* checkbox = qobject_cast<QCheckBox*>(sender());
        if (checkbox) {
            map_widget_->setRobotEnabled(checkbox->isChecked());
        }
    }
}

void MainWindow::onUpdateStatusTimer()
{
    // 定期更新状态信息
    updateConnectionStatus();
}

// 会话管理
void MainWindow::saveSettings()
{
    QSettings settings;
    
    // 保存窗口几何信息
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    
    // 保存分割器状态
    if (main_splitter_) {
        settings.setValue("mainSplitter", main_splitter_->saveState());
    }
    if (right_splitter_) {
        settings.setValue("rightSplitter", right_splitter_->saveState());
    }
    
    // 保存可视化设置
    if (path_viz_widget_) {
        auto enabled_paths = path_viz_widget_->getEnabledPaths();
        for (auto it = enabled_paths.begin(); it != enabled_paths.end(); ++it) {
            settings.setValue(QString("paths/%1").arg(it.key()), it.value());
        }
        
        auto enabled_maps = path_viz_widget_->getEnabledMaps();
        for (auto it = enabled_maps.begin(); it != enabled_maps.end(); ++it) {
            settings.setValue(QString("maps/%1").arg(it.key()), it.value());
        }
    }
}

void MainWindow::loadSettings()
{
    QSettings settings;
    
    // 恢复窗口几何信息
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    
    // 恢复分割器状态
    if (main_splitter_) {
        main_splitter_->restoreState(settings.value("mainSplitter").toByteArray());
    }
    if (right_splitter_) {
        right_splitter_->restoreState(settings.value("rightSplitter").toByteArray());
    }
}

bool MainWindow::confirmExit()
{
    if (controllers_initialized_) {
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, "确认退出",
            "确定要退出应用程序吗？\n所有未保存的设置将会丢失。",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        
        return reply == QMessageBox::Yes;
    }
    
    return true;
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
    if (main_splitter_ && right_splitter_) {
        setupResponsiveSplitters();
    }
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
