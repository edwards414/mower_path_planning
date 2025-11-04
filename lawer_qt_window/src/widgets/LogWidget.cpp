#include "LogWidget.h"
#include <QDateTime>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QScrollBar>
#include <QApplication>

LogWidget::LogWidget(QWidget *parent)
    : QWidget(parent)
    , main_layout_(nullptr)
    , log_group_(nullptr)
    , control_group_(nullptr)
    , log_display_(nullptr)
    , scroll_area_(nullptr)
    , filter_combo_(nullptr)
    , auto_scroll_checkbox_(nullptr)
    , clear_logs_btn_(nullptr)
    , save_logs_btn_(nullptr)
    , log_count_label_(nullptr)
    , max_log_entries_(1000)
    , auto_scroll_enabled_(true)
    , current_log_count_(0)
    , update_timer_(nullptr)
    , current_filter_(INFO)
{
    setupUI();
    connectSignals();
    
    // 创建更新定时器
    update_timer_ = new QTimer(this);
    update_timer_->setSingleShot(true);
    update_timer_->setInterval(100); // 100ms延迟更新
    connect(update_timer_, &QTimer::timeout, this, &LogWidget::updateLogCount);
    
    // 添加欢迎消息和示例日志
    addLog(INFO, "日志系统已启动 - 准备监控ROS2话题和服务");
    addLog(SUCCESS, "Qt界面初始化完成");
    addLog(TOPIC_SUBSCRIPTION, "等待ROS2节点连接...");
    
    // 添加一些示例日志以确保面板可见
    addLog(DEBUG, "界面组件加载完成");
    addLog(INFO, "准备接收ROS2消息和服务调用");
    
    // 调试信息
    qDebug() << "LogWidget: Initialized with" << current_log_count_ << "log entries";
    qDebug() << "LogWidget: Minimum size:" << minimumSize();
    qDebug() << "LogWidget: Size policy:" << sizePolicy().horizontalPolicy() << sizePolicy().verticalPolicy();
}

LogWidget::~LogWidget()
{
}

void LogWidget::setupUI()
{
    // 设置LogWidget的背景样式，确保可见
    setStyleSheet(
        "LogWidget {"
        "  background-color: #2b2b2b;"
        "  border: 2px solid #4CAF50;"  // 绿色边框，便于识别
        "  border-radius: 4px;"
        "}"
    );
    
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setSpacing(8);
    main_layout_->setContentsMargins(5, 5, 5, 5);

    setupControlPanel();
    setupLogDisplay();
    
    setLayout(main_layout_);
}

void LogWidget::setupControlPanel()
{
    control_group_ = new QGroupBox("日志控制 (Log Control)", this);
    QHBoxLayout* control_layout = new QHBoxLayout(control_group_);
    control_layout->setSpacing(8);
    control_layout->setContentsMargins(8, 12, 8, 8);

    // 过滤器
    QLabel* filter_label = new QLabel("过滤:", this);
    filter_combo_ = new QComboBox(this);
    filter_combo_->addItem("全部", static_cast<int>(INFO));
    filter_combo_->addItem("信息", static_cast<int>(INFO));
    filter_combo_->addItem("警告", static_cast<int>(WARNING));
    filter_combo_->addItem("错误", static_cast<int>(ERROR));
    filter_combo_->addItem("成功", static_cast<int>(SUCCESS));
    filter_combo_->addItem("话题订阅", static_cast<int>(TOPIC_SUBSCRIPTION));
    filter_combo_->addItem("服务调用", static_cast<int>(SERVICE_CALL));
    filter_combo_->addItem("调试", static_cast<int>(DEBUG));
    
    // 自动滚动
    auto_scroll_checkbox_ = new QCheckBox("自动滚动", this);
    auto_scroll_checkbox_->setChecked(auto_scroll_enabled_);
    
    // 按钮
    clear_logs_btn_ = new QPushButton("清空日志", this);
    save_logs_btn_ = new QPushButton("保存日志", this);
    
    // 日志计数
    log_count_label_ = new QLabel("日志: 0", this);
    log_count_label_->setStyleSheet("color: #888888; font-size: 11px;");
    
    // 设置按钮样式
    setupButtonStyle(clear_logs_btn_, "#FF5722");
    setupButtonStyle(save_logs_btn_, "#4CAF50");
    
    // 布局
    control_layout->addWidget(filter_label);
    control_layout->addWidget(filter_combo_);
    control_layout->addWidget(auto_scroll_checkbox_);
    control_layout->addStretch();
    control_layout->addWidget(log_count_label_);
    control_layout->addWidget(clear_logs_btn_);
    control_layout->addWidget(save_logs_btn_);
    
    main_layout_->addWidget(control_group_);
}

void LogWidget::setupLogDisplay()
{
    log_group_ = new QGroupBox("系统日志 (System Logs)", this);
    QVBoxLayout* log_layout = new QVBoxLayout(log_group_);
    log_layout->setContentsMargins(8, 12, 8, 8);

    log_display_ = new QTextEdit(this);
    log_display_->setReadOnly(true);
    log_display_->setMinimumHeight(200);
    // QTextEdit没有setMaximumBlockCount方法，我们在addLog中手动限制
    
    // 设置日志显示样式
    log_display_->setStyleSheet(
        "QTextEdit {"
        "  background-color: #1e1e1e;"
        "  color: #ffffff;"
        "  border: 1px solid #555555;"
        "  border-radius: 4px;"
        "  font-family: 'Consolas', 'Monaco', monospace;"
        "  font-size: 11px;"
        "  line-height: 1.2;"
        "  padding: 8px;"
        "}"
        "QScrollBar:vertical {"
        "  background-color: #2b2b2b;"
        "  width: 12px;"
        "  border-radius: 6px;"
        "}"
        "QScrollBar::handle:vertical {"
        "  background-color: #555555;"
        "  border-radius: 6px;"
        "  min-height: 20px;"
        "}"
        "QScrollBar::handle:vertical:hover {"
        "  background-color: #777777;"
        "}"
    );
    
    log_layout->addWidget(log_display_);
    main_layout_->addWidget(log_group_);
}

void LogWidget::setupButtonStyle(QPushButton* button, const QString& color)
{
    button->setStyleSheet(QString(
        "QPushButton {"
        "  background-color: %1;"
        "  color: white;"
        "  border: none;"
        "  padding: 6px 12px;"
        "  border-radius: 4px;"
        "  font-weight: bold;"
        "  font-size: 11px;"
        "  min-height: 24px;"
        "}"
        "QPushButton:hover {"
        "  background-color: %2;"
        "}"
        "QPushButton:pressed {"
        "  background-color: %3;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #cccccc;"
        "  color: #666666;"
        "}"
    ).arg(color)
     .arg(adjustColor(color, 20))   // 悬停时亮一些
     .arg(adjustColor(color, -20))); // 按下时暗一些
}

QString LogWidget::adjustColor(const QString& color, int adjustment)
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

void LogWidget::connectSignals()
{
    connect(filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LogWidget::onFilterChanged);
    connect(auto_scroll_checkbox_, &QCheckBox::toggled,
            this, &LogWidget::onAutoScrollToggled);
    connect(clear_logs_btn_, &QPushButton::clicked,
            this, &LogWidget::onClearLogsClicked);
    connect(save_logs_btn_, &QPushButton::clicked,
            this, &LogWidget::onSaveLogsClicked);
}

void LogWidget::addLog(LogType type, const QString& message)
{
    QString formatted_entry = formatLogEntry(type, message);
    log_entries_.append(formatted_entry);
    
    // 限制日志条目数量
    if (log_entries_.size() > max_log_entries_) {
        log_entries_.removeFirst();
    }
    
    // 更新显示
    log_display_->append(formatted_entry);
    current_log_count_++;
    
    // 自动滚动到底部
    if (auto_scroll_enabled_) {
        scrollToBottom();
    }
    
    // 延迟更新计数显示
    update_timer_->start();
}

void LogWidget::addTopicSubscription(const QString& topic_name, const QString& message_type)
{
    QString message = QString("订阅话题: %1 [%2]").arg(topic_name).arg(message_type);
    addLog(TOPIC_SUBSCRIPTION, message);
}

void LogWidget::addServiceCall(const QString& service_name, const QString& request_type, bool success)
{
    QString status = success ? "成功" : "失败";
    QString message = QString("服务调用: %1 [%2] - %3").arg(service_name).arg(request_type).arg(status);
    LogType type = success ? SERVICE_CALL : ERROR;
    addLog(type, message);
}

void LogWidget::addTopicMessage(const QString& topic_name, const QString& summary)
{
    QString message = QString("收到消息: %1 - %2").arg(topic_name).arg(summary);
    addLog(DEBUG, message);
}

QString LogWidget::formatLogEntry(LogType type, const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString type_str = getLogTypeString(type);
    QString color = getLogTypeColor(type);
    
    return QString("<span style='color: #888888;'>[%1]</span> "
                   "<span style='color: %2; font-weight: bold;'>[%3]</span> "
                   "<span style='color: #ffffff;'>%4</span>")
           .arg(timestamp)
           .arg(color)
           .arg(type_str)
           .arg(message);
}

QString LogWidget::getLogTypeString(LogType type)
{
    switch (type) {
        case INFO: return "信息";
        case WARNING: return "警告";
        case ERROR: return "错误";
        case SUCCESS: return "成功";
        case TOPIC_SUBSCRIPTION: return "订阅";
        case SERVICE_CALL: return "服务";
        case DEBUG: return "调试";
        default: return "未知";
    }
}

QString LogWidget::getLogTypeColor(LogType type)
{
    switch (type) {
        case INFO: return "#2196F3";      // 蓝色
        case WARNING: return "#FF9800";   // 橙色
        case ERROR: return "#F44336";     // 红色
        case SUCCESS: return "#4CAF50";   // 绿色
        case TOPIC_SUBSCRIPTION: return "#9C27B0"; // 紫色
        case SERVICE_CALL: return "#00BCD4";       // 青色
        case DEBUG: return "#607D8B";     // 灰蓝色
        default: return "#FFFFFF";        // 白色
    }
}

void LogWidget::scrollToBottom()
{
    QScrollBar* scrollbar = log_display_->verticalScrollBar();
    scrollbar->setValue(scrollbar->maximum());
}

void LogWidget::updateLogCount()
{
    log_count_label_->setText(QString("日志: %1").arg(current_log_count_));
}

void LogWidget::clearLogs()
{
    log_display_->clear();
    log_entries_.clear();
    current_log_count_ = 0;
    updateLogCount();
    addLog(INFO, "日志已清空");
}

void LogWidget::setMaxLogEntries(int max_entries)
{
    max_log_entries_ = max_entries;
    // QTextEdit没有setMaximumBlockCount方法，我们在addLog中手动限制条目数
}

void LogWidget::setAutoScroll(bool enabled)
{
    auto_scroll_enabled_ = enabled;
    auto_scroll_checkbox_->setChecked(enabled);
}

void LogWidget::onFilterChanged()
{
    // TODO: 实现日志过滤功能
    int filter_index = filter_combo_->currentData().toInt();
    current_filter_ = static_cast<LogType>(filter_index);
}

void LogWidget::onClearLogsClicked()
{
    clearLogs();
}

void LogWidget::onSaveLogsClicked()
{
    QString filename = QFileDialog::getSaveFileName(
        this,
        "保存日志文件",
        QString("robot_logs_%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")),
        "文本文件 (*.txt);;所有文件 (*)"
    );
    
    if (!filename.isEmpty()) {
        QFile file(filename);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream stream(&file);
            stream << log_display_->toPlainText();
            file.close();
            
            addLog(SUCCESS, QString("日志已保存到: %1").arg(filename));
        } else {
            addLog(ERROR, QString("无法保存日志文件: %1").arg(filename));
        }
    }
}

void LogWidget::onAutoScrollToggled(bool enabled)
{
    auto_scroll_enabled_ = enabled;
    if (enabled) {
        scrollToBottom();
    }
}

