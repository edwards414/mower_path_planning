#ifndef PATHVISUALIZATIONWIDGET_H
#define PATHVISUALIZATIONWIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QSlider>
#include <QLabel>
#include <QColorDialog>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QScrollArea>
#include <QFrame>
#include "../controllers/PathController.h"
#include "../controllers/MapController.h"

class PathVisualizationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PathVisualizationWidget(QWidget *parent = nullptr);
    ~PathVisualizationWidget();

    // 设置控制器
    void setPathController(PathController* controller);
    void setMapController(MapController* controller);

    // 获取当前设置
    QMap<QString, bool> getEnabledPaths() const;
    QMap<QString, bool> getEnabledMaps() const;

signals:
    void pathVisibilityChanged(const QString& path_name, bool visible);
    void mapVisibilityChanged(const QString& map_name, bool visible);
    void pathStyleChanged(const QString& path_name);
    void mapStyleChanged(const QString& map_name);
    void viewControlRequested(const QString& action);

private slots:
    void onPathCheckBoxToggled(bool checked);
    void onMapCheckBoxToggled(bool checked);
    void onPathColorButtonClicked();
    void onMapAlphaSliderChanged(int value);
    void onPathWidthChanged(double value);
    void onPathStyleChanged(const QString& style);
    void onViewControlButtonClicked();

private:
    PathController* path_controller_;
    MapController* map_controller_;

    // UI 组件
    QVBoxLayout* main_layout_;
    
    // 路径控制组
    QGroupBox* path_control_group_;
    QMap<QString, QCheckBox*> path_checkboxes_;
    QMap<QString, QPushButton*> path_color_buttons_;
    QMap<QString, QSpinBox*> path_width_spinboxes_;
    QMap<QString, QComboBox*> path_style_combos_;

    // 地图控制组
    QGroupBox* map_control_group_;
    QMap<QString, QCheckBox*> map_checkboxes_;
    QMap<QString, QSlider*> map_alpha_sliders_;
    QMap<QString, QLabel*> map_alpha_labels_;

    // 视图控制组
    QGroupBox* view_control_group_;
    QPushButton* fit_to_content_btn_;
    QPushButton* reset_view_btn_;
    QPushButton* clear_paths_btn_;
    QCheckBox* grid_checkbox_;
    QCheckBox* robot_checkbox_;

    // 统计信息组
    QGroupBox* stats_group_;
    QLabel* path_stats_label_;
    QLabel* map_stats_label_;

    // 私有方法
    void setupUI();
    void setupPathControlGroup();
    void setupMapControlGroup();
    void setupViewControlGroup();
    void setupStatsGroup();
    void connectSignals();
    void updatePathStats();
    void updateMapStats();
    
    // 路径配置
    struct PathConfig {
        QString display_name;
        QColor default_color;
        float default_width;
        QString default_style;
        bool default_enabled;
    };
    
    // 地图配置
    struct MapConfig {
        QString display_name;
        float default_alpha;
        bool default_enabled;
    };
    
    QMap<QString, PathConfig> getPathConfigs() const;
    QMap<QString, MapConfig> getMapConfigs() const;
};

#endif // PATHVISUALIZATIONWIDGET_H
