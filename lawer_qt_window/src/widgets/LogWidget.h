#ifndef LOGWIDGET_H
#define LOGWIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QScrollArea>
#include <QFrame>
#include <QTimer>
#include <QDateTime>
#include <QComboBox>
#include <QCheckBox>

class LogWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LogWidget(QWidget *parent = nullptr);
    ~LogWidget();

    // 日志类型枚举
    enum LogType {
        INFO,
        WARNING,
        ERROR,
        SUCCESS,
        TOPIC_SUBSCRIPTION,
        SERVICE_CALL,
        DEBUG
    };

public slots:
    // 日志记录方法
    void addLog(LogType type, const QString& message);
    void addTopicSubscription(const QString& topic_name, const QString& message_type);
    void addServiceCall(const QString& service_name, const QString& request_type, bool success = true);
    void addTopicMessage(const QString& topic_name, const QString& summary);
    
    // 控制方法
    void clearLogs();
    void setMaxLogEntries(int max_entries);
    void setAutoScroll(bool enabled);

private slots:
    void onFilterChanged();
    void onClearLogsClicked();
    void onSaveLogsClicked();
    void onAutoScrollToggled(bool enabled);

private:
    void setupUI();
    void setupLogDisplay();
    void setupControlPanel();
    void connectSignals();
    
    QString formatLogEntry(LogType type, const QString& message);
    QString getLogTypeString(LogType type);
    QString getLogTypeColor(LogType type);
    void scrollToBottom();
    void updateLogCount();
    QString adjustColor(const QString& color, int adjustment);
    void setupButtonStyle(QPushButton* button, const QString& color);

    // UI 组件
    QVBoxLayout* main_layout_;
    QGroupBox* log_group_;
    QGroupBox* control_group_;
    
    // 日志显示
    QTextEdit* log_display_;
    QScrollArea* scroll_area_;
    
    // 控制面板
    QComboBox* filter_combo_;
    QCheckBox* auto_scroll_checkbox_;
    QPushButton* clear_logs_btn_;
    QPushButton* save_logs_btn_;
    QLabel* log_count_label_;
    
    // 状态
    int max_log_entries_;
    bool auto_scroll_enabled_;
    int current_log_count_;
    QTimer* update_timer_;
    
    // 日志过滤
    LogType current_filter_;
    QStringList log_entries_;
};

#endif // LOGWIDGET_H
