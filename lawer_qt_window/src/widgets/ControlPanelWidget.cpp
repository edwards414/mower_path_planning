#include "ControlPanelWidget.h"
#include <QApplication>
#include <QDateTime>
#include <QScrollBar>

ControlPanelWidget::ControlPanelWidget(QWidget *parent)
    : QWidget(parent)
    , service_controller_(nullptr)
    , main_layout_(nullptr)
    , progress_timer_(new QTimer(this))
    , is_service_running_(false)
{
    setupUI();
    connectSignals();
    
    // 设置进度条定时器
    progress_timer_->setInterval(100);
    connect(progress_timer_, &QTimer::timeout, this, &ControlPanelWidget::onProgressTimeout);
}

ControlPanelWidget::~ControlPanelWidget()
{
}

void ControlPanelWidget::setupUI()
{
    // 创建滚动区域
    QScrollArea* scroll_area = new QScrollArea(this);
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area->setFrameShape(QFrame::NoFrame);
    
    // 创建滚动内容widget
    QWidget* scroll_content = new QWidget();
    scroll_content->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    
    main_layout_ = new QVBoxLayout(scroll_content);
    main_layout_->setSpacing(10);
    main_layout_->setContentsMargins(10, 10, 10, 10);

    // 创建各个功能组
    setupMapManagementGroup();
    setupPathRecordGroup();
    setupCoveragePlanningGroup();
    setupStatusGroup();

    // 添加弹性空间
    main_layout_->addStretch();
    
    // 设置滚动内容
    scroll_area->setWidget(scroll_content);
    
    // 主布局
    QVBoxLayout* main_widget_layout = new QVBoxLayout(this);
    main_widget_layout->setContentsMargins(0, 0, 0, 0);
    main_widget_layout->addWidget(scroll_area);
    setLayout(main_widget_layout);
}

void ControlPanelWidget::setupMapManagementGroup()
{
    map_management_group_ = new QGroupBox("地图管理 (Map Management)", this);
    QGridLayout* layout = new QGridLayout(map_management_group_);
    layout->setSpacing(8);
    layout->setContentsMargins(10, 15, 10, 10);

    // 创建按钮
    create_free_space_btn_ = new QPushButton("生成FreeSpace", this);
    create_risk_map_btn_ = new QPushButton("生成RiskSpace", this);
    create_chennal_map_btn_ = new QPushButton("建立Chennal Map", this);
    create_bcd_btn_ = new QPushButton("建立BCD", this);
    get_zone_list_btn_ = new QPushButton("获取区域列表", this);

    // 设置按钮最小尺寸
    QList<QPushButton*> buttons = {create_free_space_btn_, create_risk_map_btn_, 
                                   create_chennal_map_btn_, create_bcd_btn_, get_zone_list_btn_};
    for (auto* btn : buttons) {
        btn->setMinimumSize(120, 35);
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    // 设置按钮样式
    setupButtonStyle(create_free_space_btn_, "#4CAF50");
    setupButtonStyle(create_risk_map_btn_, "#FF9800");
    setupButtonStyle(create_chennal_map_btn_, "#2196F3");
    setupButtonStyle(create_bcd_btn_, "#9C27B0");
    setupButtonStyle(get_zone_list_btn_, "#607D8B");

    // 布局按钮
    layout->addWidget(create_free_space_btn_, 0, 0);
    layout->addWidget(create_risk_map_btn_, 0, 1);
    layout->addWidget(create_chennal_map_btn_, 1, 0);
    layout->addWidget(create_bcd_btn_, 1, 1);
    layout->addWidget(get_zone_list_btn_, 2, 0, 1, 2);

    main_layout_->addWidget(map_management_group_);
}

void ControlPanelWidget::setupPathRecordGroup()
{
    path_record_group_ = new QGroupBox("路径记录 (Path Recording)", this);
    QVBoxLayout* main_layout = new QVBoxLayout(path_record_group_);
    main_layout->setSpacing(8);
    main_layout->setContentsMargins(10, 15, 10, 10);

    // Zone Recording
    record_zone_start_btn_ = new QPushButton("区域记录开始", this);
    record_zone_end_btn_ = new QPushButton("区域记录结束", this);
    save_zone_list_btn_ = new QPushButton("保存区域", this);
    load_zone_list_btn_ = new QPushButton("加载区域", this);

    // Risk Zone Recording
    risk_zone_start_btn_ = new QPushButton("Risk区域开始", this);
    risk_zone_end_btn_ = new QPushButton("Risk区域结束", this);
    risk_zone_save_btn_ = new QPushButton("保存Risk区域", this);
    get_record_info_btn_ = new QPushButton("获取记录信息", this);

    // Chennal Recording
    chennal_record_start_btn_ = new QPushButton("Chennal记录开始", this);
    chennal_record_end_btn_ = new QPushButton("Chennal记录结束", this);
    get_chennal_list_btn_ = new QPushButton("获取Chennal列表", this);

    // 设置按钮最小尺寸和响应式策略
    QList<QPushButton*> buttons = {
        record_zone_start_btn_, record_zone_end_btn_, save_zone_list_btn_, load_zone_list_btn_,
        risk_zone_start_btn_, risk_zone_end_btn_, risk_zone_save_btn_, get_record_info_btn_,
        chennal_record_start_btn_, chennal_record_end_btn_, get_chennal_list_btn_
    };
    for (auto* btn : buttons) {
        btn->setMinimumSize(80, 32);  // 减小最小宽度以适应小屏幕
        btn->setMaximumHeight(40);    // 限制最大高度
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    // 设置按钮样式
    setupButtonStyle(record_zone_start_btn_, "#4CAF50");
    setupButtonStyle(record_zone_end_btn_, "#F44336");
    setupButtonStyle(save_zone_list_btn_, "#2196F3");
    setupButtonStyle(load_zone_list_btn_, "#FF9800");
    setupButtonStyle(risk_zone_start_btn_, "#E91E63");
    setupButtonStyle(risk_zone_end_btn_, "#9C27B0");
    setupButtonStyle(risk_zone_save_btn_, "#673AB7");
    setupButtonStyle(get_record_info_btn_, "#607D8B");
    setupButtonStyle(chennal_record_start_btn_, "#009688");
    setupButtonStyle(chennal_record_end_btn_, "#795548");
    setupButtonStyle(get_chennal_list_btn_, "#FFC107");

    // 创建响应式网格布局
    createResponsiveButtonGrid(main_layout, {
        {record_zone_start_btn_, record_zone_end_btn_, save_zone_list_btn_, load_zone_list_btn_},
        {risk_zone_start_btn_, risk_zone_end_btn_, risk_zone_save_btn_, get_record_info_btn_},
        {chennal_record_start_btn_, chennal_record_end_btn_, get_chennal_list_btn_}
    });

    main_layout_->addWidget(path_record_group_);
}

void ControlPanelWidget::setupCoveragePlanningGroup()
{
    coverage_planning_group_ = new QGroupBox("覆盖路径规划 (Coverage Planning)", this);
    QHBoxLayout* layout = new QHBoxLayout(coverage_planning_group_);
    layout->setSpacing(8);
    layout->setContentsMargins(10, 15, 10, 10);

    // 创建按钮
    generate_coverage_path_btn_ = new QPushButton("生成覆盖路径", this);
    waypoint_pub_btn_ = new QPushButton("发布航点", this);
    cancel_nav2_btn_ = new QPushButton("取消导航", this);

    // 设置按钮最小尺寸
    QList<QPushButton*> buttons = {generate_coverage_path_btn_, waypoint_pub_btn_, cancel_nav2_btn_};
    for (auto* btn : buttons) {
        btn->setMinimumSize(120, 35);
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    // 设置按钮样式
    setupButtonStyle(generate_coverage_path_btn_, "#4CAF50");
    setupButtonStyle(waypoint_pub_btn_, "#2196F3");
    setupButtonStyle(cancel_nav2_btn_, "#F44336");

    // 布局按钮
    layout->addWidget(generate_coverage_path_btn_);
    layout->addWidget(waypoint_pub_btn_);
    layout->addWidget(cancel_nav2_btn_);

    main_layout_->addWidget(coverage_planning_group_);
}

void ControlPanelWidget::setupStatusGroup()
{
    status_group_ = new QGroupBox("状态信息 (Status)", this);
    QVBoxLayout* layout = new QVBoxLayout(status_group_);

    // 状态标签
    status_label_ = new QLabel("就绪", this);
    status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");

    // 进度条
    progress_bar_ = new QProgressBar(this);
    progress_bar_->setVisible(false);

    // 消息显示区域
    message_display_ = new QTextEdit(this);
    message_display_->setMaximumHeight(150);
    message_display_->setReadOnly(true);
    message_display_->setStyleSheet(
        "QTextEdit {"
        "  background-color: #f5f5f5;"
        "  border: 1px solid #ddd;"
        "  border-radius: 4px;"
        "  padding: 5px;"
        "  font-family: 'Consolas', 'Monaco', monospace;"
        "  font-size: 10px;"
        "}"
    );

    layout->addWidget(status_label_);
    layout->addWidget(progress_bar_);
    layout->addWidget(message_display_);

    main_layout_->addWidget(status_group_);
}

void ControlPanelWidget::setupButtonStyle(QPushButton* button, const QString& color)
{
    // 计算反色
    QColor base_color(color);
    QString hover_color = getContrastColor(color);      // 悬停时使用反色
    QString pressed_color = adjustColor(color, -40);    // 按下时使用更暗的颜色
    
    button->setStyleSheet(QString(
        "QPushButton {"
        "  background-color: %1;"
        "  color: white;"
        "  border: 2px solid transparent;"
        "  padding: 10px 16px;"
        "  border-radius: 6px;"
        "  font-weight: bold;"
        "  font-size: 12px;"
        "  min-height: 32px;"
        "  text-align: center;"
        "}"
        "QPushButton:hover {"
        "  background-color: %2;"
        "  color: %1;"
        "  border: 2px solid %1;"
        "}"
        "QPushButton:pressed {"
        "  background-color: %3;"
        "  color: white;"
        "  border: 2px solid %3;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #cccccc;"
        "  color: #666666;"
        "  border: 2px solid transparent;"
        "}"
    ).arg(color)
     .arg(hover_color)      // 悬停时反色背景
     .arg(pressed_color));  // 按下时深色
}

QString ControlPanelWidget::adjustColor(const QString& color, int adjustment)
{
    QColor qcolor(color);
    int r = qBound(0, qcolor.red() + adjustment, 255);
    int g = qBound(0, qcolor.green() + adjustment, 255);
    int b = qBound(0, qcolor.blue() + adjustment, 255);
    return QString("#%1%2%3")
           .arg(r, 2, 16, QChar('0'))
           .arg(g, 2, 16, QChar('0'))
           .arg(b, 2, 16, QChar('0'));
}

QString ControlPanelWidget::getContrastColor(const QString& color)
{
    QColor qcolor(color);
    
    // 计算亮度 (使用相对亮度公式)
    double luminance = (0.299 * qcolor.red() + 0.587 * qcolor.green() + 0.114 * qcolor.blue()) / 255.0;
    
    if (luminance > 0.5) {
        // 如果颜色较亮，返回深色背景
        return "#2b2b2b";
    } else {
        // 如果颜色较暗，返回亮色背景
        return "#f0f0f0";
    }
}

void ControlPanelWidget::createResponsiveButtonGrid(QVBoxLayout* parent_layout, 
                                                   const QVector<QVector<QPushButton*>>& button_rows)
{
    for (const auto& row : button_rows) {
        // 创建水平布局
        QHBoxLayout* row_layout = new QHBoxLayout();
        row_layout->setSpacing(6);
        
        for (auto* button : row) {
            row_layout->addWidget(button);
        }
        
        parent_layout->addLayout(row_layout);
    }
}

void ControlPanelWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    
    // 响应式调整按钮文字
    adjustButtonTextForWidth(width());
}

void ControlPanelWidget::adjustButtonTextForWidth(int width)
{
    // 根据宽度调整按钮文字显示
    bool use_short_text = width < 400;
    
    if (use_short_text) {
        // 使用简短文字
        record_zone_start_btn_->setText("区域开始");
        record_zone_end_btn_->setText("区域结束");
        save_zone_list_btn_->setText("保存");
        load_zone_list_btn_->setText("加载");
        risk_zone_start_btn_->setText("Risk开始");
        risk_zone_end_btn_->setText("Risk结束");
        risk_zone_save_btn_->setText("Risk保存");
        get_record_info_btn_->setText("获取信息");
        chennal_record_start_btn_->setText("Ch开始");
        chennal_record_end_btn_->setText("Ch结束");
        get_chennal_list_btn_->setText("Ch列表");
        
        create_free_space_btn_->setText("FreeSpace");
        create_risk_map_btn_->setText("RiskSpace");
        create_chennal_map_btn_->setText("Chennal");
        create_bcd_btn_->setText("BCD");
        get_zone_list_btn_->setText("区域列表");
        
        generate_coverage_path_btn_->setText("生成路径");
        waypoint_pub_btn_->setText("发布点");
        cancel_nav2_btn_->setText("取消");
    } else {
        // 使用完整文字
        record_zone_start_btn_->setText("区域记录开始");
        record_zone_end_btn_->setText("区域记录结束");
        save_zone_list_btn_->setText("保存区域");
        load_zone_list_btn_->setText("加载区域");
        risk_zone_start_btn_->setText("Risk区域开始");
        risk_zone_end_btn_->setText("Risk区域结束");
        risk_zone_save_btn_->setText("保存Risk区域");
        get_record_info_btn_->setText("获取记录信息");
        chennal_record_start_btn_->setText("Chennal记录开始");
        chennal_record_end_btn_->setText("Chennal记录结束");
        get_chennal_list_btn_->setText("获取Chennal列表");
        
        create_free_space_btn_->setText("生成FreeSpace");
        create_risk_map_btn_->setText("生成RiskSpace");
        create_chennal_map_btn_->setText("建立Chennal Map");
        create_bcd_btn_->setText("建立BCD");
        get_zone_list_btn_->setText("获取区域列表");
        
        generate_coverage_path_btn_->setText("生成覆盖路径");
        waypoint_pub_btn_->setText("发布航点");
        cancel_nav2_btn_->setText("取消导航");
    }
}

void ControlPanelWidget::connectSignals()
{
    qDebug() << "ControlPanelWidget: connectSignals() called";
    qDebug() << "ControlPanelWidget: create_free_space_btn_:" << create_free_space_btn_;
    
    // Map Management Services
    connect(create_free_space_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onCreateFreeSpaceClicked);
    connect(create_risk_map_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onCreateRiskMapClicked);
    connect(create_chennal_map_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onCreateChennalMapClicked);
    connect(create_bcd_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onCreateZoneCellDecompositionClicked);
    connect(get_zone_list_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onGetRecordZoneListSrvClicked);

    // Path Record Services
    connect(record_zone_start_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onRecordZoneStartClicked);
    connect(record_zone_end_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onRecordZoneEndClicked);
    connect(risk_zone_start_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onRiskZoneStartClicked);
    connect(risk_zone_end_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onRiskZoneEndClicked);
    connect(save_zone_list_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onSaveZoneListClicked);
    connect(risk_zone_save_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onRiskZoneSaveClicked);
    connect(load_zone_list_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onLoadZoneListClicked);
    connect(get_record_info_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onGetRecordZoneInfoClicked);
    connect(chennal_record_start_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onChennalRecordStartClicked);
    connect(chennal_record_end_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onChennalRecordEndClicked);
    connect(get_chennal_list_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onGetChennalPathListClicked);

    // Coverage Planning Services
    connect(generate_coverage_path_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onGenerateCoveragePathClicked);
    connect(waypoint_pub_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onWaypointPubClicked);
    connect(cancel_nav2_btn_, &QPushButton::clicked, this, &ControlPanelWidget::onCancelNav2Clicked);
}

void ControlPanelWidget::setServiceController(ServiceController* controller)
{
    qDebug() << "ControlPanelWidget: setServiceController called with controller:" << controller;
    
    if (service_controller_) {
        // 断开旧的连接
        disconnect(service_controller_, nullptr, this, nullptr);
    }

    service_controller_ = controller;

    if (service_controller_) {
        qDebug() << "ControlPanelWidget: ServiceController set successfully";
        // 连接新的信号
        connect(service_controller_, &ServiceController::serviceCallStarted,
                this, &ControlPanelWidget::onServiceCallStarted);
        connect(service_controller_, &ServiceController::serviceCallCompleted,
                this, &ControlPanelWidget::onServiceCallCompleted);
        connect(service_controller_, &ServiceController::serviceCallFailed,
                this, &ControlPanelWidget::onServiceCallFailed);

        updateButtonStates();
    } else {
        qDebug() << "ControlPanelWidget: ServiceController is nullptr!";
    }
}

void ControlPanelWidget::setEnabled(bool enabled)
{
    QWidget::setEnabled(enabled);
    updateButtonStates();
}

void ControlPanelWidget::showMessage(const QString& message, bool is_error)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString color = is_error ? "red" : "black";
    QString formatted_message = QString("<span style='color: %1'>[%2] %3</span>")
                               .arg(color)
                               .arg(timestamp)
                               .arg(message);
    
    message_display_->append(formatted_message);
    
    // 自动滚动到底部
    QScrollBar* scrollbar = message_display_->verticalScrollBar();
    scrollbar->setValue(scrollbar->maximum());
}

void ControlPanelWidget::clearMessages()
{
    message_display_->clear();
}

void ControlPanelWidget::updateButtonStates()
{
    qDebug() << "=== ControlPanelWidget::updateButtonStates() ===";
    qDebug() << "Widget enabled:" << isEnabled();
    qDebug() << "ServiceController exists:" << (service_controller_ != nullptr);
    qDebug() << "ServiceController initialized:" << (service_controller_ ? service_controller_->isInitialized() : false);
    qDebug() << "Service running:" << is_service_running_;
    
    bool enabled = isEnabled() && service_controller_ && 
                  service_controller_->isInitialized() && !is_service_running_;
    
    qDebug() << "Final button enabled state:" << enabled;
    
    // 更新所有按钮状态
    create_free_space_btn_->setEnabled(enabled);
    create_risk_map_btn_->setEnabled(enabled);
    create_chennal_map_btn_->setEnabled(enabled);
    create_bcd_btn_->setEnabled(enabled);
    get_zone_list_btn_->setEnabled(enabled);
    
    record_zone_start_btn_->setEnabled(enabled);
    record_zone_end_btn_->setEnabled(enabled);
    risk_zone_start_btn_->setEnabled(enabled);
    risk_zone_end_btn_->setEnabled(enabled);
    save_zone_list_btn_->setEnabled(enabled);
    risk_zone_save_btn_->setEnabled(enabled);
    load_zone_list_btn_->setEnabled(enabled);
    get_record_info_btn_->setEnabled(enabled);
    chennal_record_start_btn_->setEnabled(enabled);
    chennal_record_end_btn_->setEnabled(enabled);
    get_chennal_list_btn_->setEnabled(enabled);
    
    generate_coverage_path_btn_->setEnabled(enabled);
    waypoint_pub_btn_->setEnabled(enabled);
    cancel_nav2_btn_->setEnabled(enabled);
    
    qDebug() << "Button states updated. Example button (create_free_space_btn_) enabled:" << create_free_space_btn_->isEnabled();
}

void ControlPanelWidget::setButtonBusy(QPushButton* button, bool busy)
{
    if (busy) {
        button->setText(button->text() + " ...");
        button->setEnabled(false);
    } else {
        QString text = button->text();
        if (text.endsWith(" ...")) {
            text.chop(4);
            button->setText(text);
        }
        updateButtonStates();
    }
}

void ControlPanelWidget::onServiceCallStarted(const QString& serviceName)
{
    current_service_ = serviceName;
    is_service_running_ = true;
    
    status_label_->setText(QString("正在执行: %1").arg(serviceName));
    status_label_->setStyleSheet("QLabel { color: orange; font-weight: bold; }");
    
    progress_bar_->setVisible(true);
    progress_bar_->setRange(0, 0);  // 无限进度条
    progress_timer_->start();
    
    updateButtonStates();
    showMessage(QString("开始执行服务: %1").arg(serviceName));
}

void ControlPanelWidget::onServiceCallCompleted(const QString& serviceName, bool success, const QString& message)
{
    current_service_.clear();
    is_service_running_ = false;
    
    status_label_->setText("就绪");
    status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    
    progress_bar_->setVisible(false);
    progress_timer_->stop();
    
    updateButtonStates();
    showMessage(QString("服务完成: %1 - %2").arg(serviceName).arg(message), !success);
}

void ControlPanelWidget::onServiceCallFailed(const QString& serviceName, const QString& error)
{
    current_service_.clear();
    is_service_running_ = false;
    
    status_label_->setText("错误");
    status_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    
    progress_bar_->setVisible(false);
    progress_timer_->stop();
    
    updateButtonStates();
    showMessage(QString("服务失败: %1 - %2").arg(serviceName).arg(error), true);
}

void ControlPanelWidget::onProgressTimeout()
{
    // 进度条动画效果
    if (progress_bar_->isVisible()) {
        progress_bar_->setValue((progress_bar_->value() + 1) % 100);
    }
}

// Map Management Service Slots
void ControlPanelWidget::onCreateFreeSpaceClicked()
{
    qDebug() << "ControlPanelWidget: onCreateFreeSpaceClicked() called";
    if (service_controller_) {
        qDebug() << "ControlPanelWidget: Calling service_controller_->createFreeSpace()";
        service_controller_->createFreeSpace();
    } else {
        qDebug() << "ControlPanelWidget: service_controller_ is nullptr!";
    }
}

void ControlPanelWidget::onCreateRiskMapClicked()
{
    if (service_controller_) {
        service_controller_->createRiskMap();
    }
}

void ControlPanelWidget::onCreateChennalMapClicked()
{
    if (service_controller_) {
        service_controller_->createChennalMap();
    }
}

void ControlPanelWidget::onCreateZoneCellDecompositionClicked()
{
    if (service_controller_) {
        service_controller_->createZoneCellDecomposition();
    }
}

void ControlPanelWidget::onGetRecordZoneListSrvClicked()
{
    if (service_controller_) {
        service_controller_->getRecordZoneListSrv();
    }
}

// Path Record Service Slots
void ControlPanelWidget::onRecordZoneStartClicked()
{
    qDebug() << "ControlPanelWidget: onRecordZoneStartClicked() called";
    if (service_controller_) {
        qDebug() << "ControlPanelWidget: Calling service_controller_->recordZoneStart()";
        service_controller_->recordZoneStart();
    } else {
        qDebug() << "ControlPanelWidget: service_controller_ is nullptr!";
    }
}

void ControlPanelWidget::onRecordZoneEndClicked()
{
    if (service_controller_) {
        service_controller_->recordZoneEnd();
    }
}

void ControlPanelWidget::onRiskZoneStartClicked()
{
    if (service_controller_) {
        service_controller_->riskZoneStart();
    }
}

void ControlPanelWidget::onRiskZoneEndClicked()
{
    if (service_controller_) {
        service_controller_->riskZoneEnd();
    }
}

void ControlPanelWidget::onSaveZoneListClicked()
{
    if (service_controller_) {
        service_controller_->saveZoneList();
    }
}

void ControlPanelWidget::onRiskZoneSaveClicked()
{
    if (service_controller_) {
        service_controller_->riskZoneSave();
    }
}

void ControlPanelWidget::onLoadZoneListClicked()
{
    if (service_controller_) {
        service_controller_->loadZoneList();
    }
}

void ControlPanelWidget::onGetRecordZoneInfoClicked()
{
    if (service_controller_) {
        service_controller_->getRecordZoneInfo();
    }
}

void ControlPanelWidget::onChennalRecordStartClicked()
{
    if (service_controller_) {
        service_controller_->chennalRecordStart();
    }
}

void ControlPanelWidget::onChennalRecordEndClicked()
{
    if (service_controller_) {
        service_controller_->chennalRecordEnd();
    }
}

void ControlPanelWidget::onGetChennalPathListClicked()
{
    if (service_controller_) {
        service_controller_->getChennalPathList();
    }
}

// Coverage Planning Service Slots
void ControlPanelWidget::onGenerateCoveragePathClicked()
{
    if (service_controller_) {
        service_controller_->generateCoveragePath();
    }
}

void ControlPanelWidget::onWaypointPubClicked()
{
    if (service_controller_) {
        service_controller_->waypointPub();
    }
}

void ControlPanelWidget::onCancelNav2Clicked()
{
    if (service_controller_) {
        service_controller_->cancelNav2();
    }
}
