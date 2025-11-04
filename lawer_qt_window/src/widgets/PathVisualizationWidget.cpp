#include "PathVisualizationWidget.h"
#include <QDebug>
#include <QGridLayout>

PathVisualizationWidget::PathVisualizationWidget(QWidget *parent)
    : QWidget(parent)
    , path_controller_(nullptr)
    , map_controller_(nullptr)
    , main_layout_(nullptr)
{
    setupUI();
    connectSignals();
}

PathVisualizationWidget::~PathVisualizationWidget()
{
}

void PathVisualizationWidget::setupUI()
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

    setupPathControlGroup();
    setupMapControlGroup();
    setupViewControlGroup();
    setupStatsGroup();

    main_layout_->addStretch();
    
    // 设置滚动内容
    scroll_area->setWidget(scroll_content);
    
    // 主布局
    QVBoxLayout* main_widget_layout = new QVBoxLayout(this);
    main_widget_layout->setContentsMargins(0, 0, 0, 0);
    main_widget_layout->addWidget(scroll_area);
    setLayout(main_widget_layout);
}

void PathVisualizationWidget::setupPathControlGroup()
{
    path_control_group_ = new QGroupBox("路径显示控制 (Path Control)", this);
    QGridLayout* layout = new QGridLayout(path_control_group_);

    auto path_configs = getPathConfigs();
    int row = 0;

    // 添加表头
    layout->addWidget(new QLabel("路径"), row, 0);
    layout->addWidget(new QLabel("显示"), row, 1);
    layout->addWidget(new QLabel("颜色"), row, 2);
    layout->addWidget(new QLabel("宽度"), row, 3);
    layout->addWidget(new QLabel("样式"), row, 4);
    row++;

    for (auto it = path_configs.begin(); it != path_configs.end(); ++it) {
        const QString& path_name = it.key();
        const PathConfig& config = it.value();

        // 路径名称标签
        QLabel* name_label = new QLabel(config.display_name, this);
        layout->addWidget(name_label, row, 0);

        // 显示复选框
        QCheckBox* checkbox = new QCheckBox(this);
        checkbox->setChecked(config.default_enabled);
        checkbox->setObjectName(path_name);
        path_checkboxes_[path_name] = checkbox;
        layout->addWidget(checkbox, row, 1);

        // 颜色按钮
        QPushButton* color_btn = new QPushButton(this);
        color_btn->setFixedSize(30, 20);
        color_btn->setStyleSheet(QString("background-color: %1; border: 1px solid black;")
                                .arg(config.default_color.name()));
        color_btn->setObjectName(path_name);
        path_color_buttons_[path_name] = color_btn;
        layout->addWidget(color_btn, row, 2);

        // 线宽调节
        QSpinBox* width_spinbox = new QSpinBox(this);
        width_spinbox->setRange(1, 20);
        width_spinbox->setValue(static_cast<int>(config.default_width * 100));
        width_spinbox->setSuffix(" px");
        width_spinbox->setObjectName(path_name);
        path_width_spinboxes_[path_name] = width_spinbox;
        layout->addWidget(width_spinbox, row, 3);

        // 样式选择
        QComboBox* style_combo = new QComboBox(this);
        style_combo->addItems({"Lines", "Billboards"});
        style_combo->setCurrentText(config.default_style);
        style_combo->setObjectName(path_name);
        path_style_combos_[path_name] = style_combo;
        layout->addWidget(style_combo, row, 4);

        row++;
    }

    main_layout_->addWidget(path_control_group_);
}

void PathVisualizationWidget::setupMapControlGroup()
{
    map_control_group_ = new QGroupBox("地图显示控制 (Map Control)", this);
    QGridLayout* layout = new QGridLayout(map_control_group_);

    auto map_configs = getMapConfigs();
    int row = 0;

    // 添加表头
    layout->addWidget(new QLabel("地图层"), row, 0);
    layout->addWidget(new QLabel("显示"), row, 1);
    layout->addWidget(new QLabel("透明度"), row, 2);
    row++;

    for (auto it = map_configs.begin(); it != map_configs.end(); ++it) {
        const QString& map_name = it.key();
        const MapConfig& config = it.value();

        // 地图名称标签
        QLabel* name_label = new QLabel(config.display_name, this);
        layout->addWidget(name_label, row, 0);

        // 显示复选框
        QCheckBox* checkbox = new QCheckBox(this);
        checkbox->setChecked(config.default_enabled);
        checkbox->setObjectName(map_name);
        map_checkboxes_[map_name] = checkbox;
        layout->addWidget(checkbox, row, 1);

        // 透明度滑块
        QHBoxLayout* alpha_layout = new QHBoxLayout();
        QSlider* alpha_slider = new QSlider(Qt::Horizontal, this);
        alpha_slider->setRange(0, 100);
        alpha_slider->setValue(static_cast<int>(config.default_alpha * 100));
        alpha_slider->setObjectName(map_name);
        map_alpha_sliders_[map_name] = alpha_slider;

        QLabel* alpha_label = new QLabel(QString::number(config.default_alpha, 'f', 2), this);
        alpha_label->setMinimumWidth(30);
        map_alpha_labels_[map_name] = alpha_label;

        alpha_layout->addWidget(alpha_slider);
        alpha_layout->addWidget(alpha_label);
        layout->addLayout(alpha_layout, row, 2);

        row++;
    }

    main_layout_->addWidget(map_control_group_);
}

void PathVisualizationWidget::setupViewControlGroup()
{
    view_control_group_ = new QGroupBox("视图控制 (View Control)", this);
    QVBoxLayout* layout = new QVBoxLayout(view_control_group_);

    // 视图操作按钮
    QHBoxLayout* button_layout = new QHBoxLayout();
    
    fit_to_content_btn_ = new QPushButton("适应内容", this);
    fit_to_content_btn_->setObjectName("fit_to_content");
    button_layout->addWidget(fit_to_content_btn_);

    reset_view_btn_ = new QPushButton("重置视图", this);
    reset_view_btn_->setObjectName("reset_view");
    button_layout->addWidget(reset_view_btn_);

    clear_paths_btn_ = new QPushButton("清除路径", this);
    clear_paths_btn_->setObjectName("clear_paths");
    button_layout->addWidget(clear_paths_btn_);

    layout->addLayout(button_layout);

    // 显示选项
    QHBoxLayout* option_layout = new QHBoxLayout();
    
    grid_checkbox_ = new QCheckBox("显示网格", this);
    grid_checkbox_->setChecked(true);
    grid_checkbox_->setObjectName("grid");
    option_layout->addWidget(grid_checkbox_);

    robot_checkbox_ = new QCheckBox("显示机器人", this);
    robot_checkbox_->setChecked(true);
    robot_checkbox_->setObjectName("robot");
    option_layout->addWidget(robot_checkbox_);

    layout->addLayout(option_layout);

    main_layout_->addWidget(view_control_group_);
}

void PathVisualizationWidget::setupStatsGroup()
{
    stats_group_ = new QGroupBox("统计信息 (Statistics)", this);
    QVBoxLayout* layout = new QVBoxLayout(stats_group_);

    path_stats_label_ = new QLabel("路径统计: 0 条路径", this);
    path_stats_label_->setStyleSheet("QLabel { color: #666; font-size: 12px; }");
    layout->addWidget(path_stats_label_);

    map_stats_label_ = new QLabel("地图统计: 0 个地图层", this);
    map_stats_label_->setStyleSheet("QLabel { color: #666; font-size: 12px; }");
    layout->addWidget(map_stats_label_);

    main_layout_->addWidget(stats_group_);
}

void PathVisualizationWidget::connectSignals()
{
    // 路径控制信号
    for (auto it = path_checkboxes_.begin(); it != path_checkboxes_.end(); ++it) {
        connect(it.value(), &QCheckBox::toggled, this, &PathVisualizationWidget::onPathCheckBoxToggled);
    }

    for (auto it = path_color_buttons_.begin(); it != path_color_buttons_.end(); ++it) {
        connect(it.value(), &QPushButton::clicked, this, &PathVisualizationWidget::onPathColorButtonClicked);
    }

    for (auto it = path_width_spinboxes_.begin(); it != path_width_spinboxes_.end(); ++it) {
        connect(it.value(), QOverload<int>::of(&QSpinBox::valueChanged), 
                [this](int value) { onPathWidthChanged(value / 100.0); });
    }

    for (auto it = path_style_combos_.begin(); it != path_style_combos_.end(); ++it) {
        connect(it.value(), &QComboBox::currentTextChanged, this, &PathVisualizationWidget::onPathStyleChanged);
    }

    // 地图控制信号
    for (auto it = map_checkboxes_.begin(); it != map_checkboxes_.end(); ++it) {
        connect(it.value(), &QCheckBox::toggled, this, &PathVisualizationWidget::onMapCheckBoxToggled);
    }

    for (auto it = map_alpha_sliders_.begin(); it != map_alpha_sliders_.end(); ++it) {
        connect(it.value(), &QSlider::valueChanged, this, &PathVisualizationWidget::onMapAlphaSliderChanged);
    }

    // 视图控制信号
    connect(fit_to_content_btn_, &QPushButton::clicked, this, &PathVisualizationWidget::onViewControlButtonClicked);
    connect(reset_view_btn_, &QPushButton::clicked, this, &PathVisualizationWidget::onViewControlButtonClicked);
    connect(clear_paths_btn_, &QPushButton::clicked, this, &PathVisualizationWidget::onViewControlButtonClicked);
    connect(grid_checkbox_, &QCheckBox::toggled, this, &PathVisualizationWidget::onViewControlButtonClicked);
    connect(robot_checkbox_, &QCheckBox::toggled, this, &PathVisualizationWidget::onViewControlButtonClicked);
}

QMap<QString, PathVisualizationWidget::PathConfig> PathVisualizationWidget::getPathConfigs() const
{
    QMap<QString, PathConfig> configs;
    
    configs["global_plan"] = {"全局规划", QColor(25, 255, 0), 0.03f, "Lines", true};
    configs["chennal_path"] = {"通道路径", QColor(87, 227, 137), 0.08f, "Billboards", true};
    configs["coverage_path"] = {"覆盖路径", QColor(25, 255, 0), 0.03f, "Lines", true};
    configs["record_path"] = {"记录路径", QColor(229, 165, 10), 0.10f, "Billboards", true};
    configs["plan"] = {"规划路径", QColor(51, 209, 122), 0.03f, "Lines", true};
    configs["split_path"] = {"分割路径", QColor(25, 255, 255), 0.06f, "Billboards", true};
    
    return configs;
}

QMap<QString, PathVisualizationWidget::MapConfig> PathVisualizationWidget::getMapConfigs() const
{
    QMap<QString, MapConfig> configs;
    
    configs["chennal_map"] = {"通道地图", 0.5f, true};
    configs["free_space"] = {"自由空间", 0.5f, true};
    configs["free_space_inflated"] = {"自由空间(膨胀)", 0.3f, false};
    configs["risk_map"] = {"风险地图", 0.2f, true};
    configs["risk_map_inflated"] = {"风险地图(膨胀)", 0.3f, false};
    configs["chennal_map_inflated"] = {"通道地图(膨胀)", 0.4f, false};
    configs["local_costmap"] = {"局部代价地图", 0.5f, true};
    configs["global_costmap"] = {"全局代价地图", 0.7f, true};
    
    return configs;
}

void PathVisualizationWidget::setPathController(PathController* controller)
{
    if (path_controller_) {
        disconnect(path_controller_, nullptr, this, nullptr);
    }
    
    path_controller_ = controller;
    
    if (path_controller_) {
        connect(path_controller_, &PathController::pathDataUpdated,
                this, &PathVisualizationWidget::updatePathStats);
        connect(path_controller_, &PathController::waypointDataUpdated,
                this, &PathVisualizationWidget::updatePathStats);
    }
    
    updatePathStats();
}

void PathVisualizationWidget::setMapController(MapController* controller)
{
    if (map_controller_) {
        disconnect(map_controller_, nullptr, this, nullptr);
    }
    
    map_controller_ = controller;
    
    if (map_controller_) {
        connect(map_controller_, &MapController::mapDataUpdated,
                this, &PathVisualizationWidget::updateMapStats);
        connect(map_controller_, &MapController::zoneDataUpdated,
                this, &PathVisualizationWidget::updateMapStats);
    }
    
    updateMapStats();
}

QMap<QString, bool> PathVisualizationWidget::getEnabledPaths() const
{
    QMap<QString, bool> enabled_paths;
    for (auto it = path_checkboxes_.begin(); it != path_checkboxes_.end(); ++it) {
        enabled_paths[it.key()] = it.value()->isChecked();
    }
    return enabled_paths;
}

QMap<QString, bool> PathVisualizationWidget::getEnabledMaps() const
{
    QMap<QString, bool> enabled_maps;
    for (auto it = map_checkboxes_.begin(); it != map_checkboxes_.end(); ++it) {
        enabled_maps[it.key()] = it.value()->isChecked();
    }
    return enabled_maps;
}

void PathVisualizationWidget::onPathCheckBoxToggled(bool checked)
{
    QCheckBox* checkbox = qobject_cast<QCheckBox*>(sender());
    if (checkbox) {
        QString path_name = checkbox->objectName();
        emit pathVisibilityChanged(path_name, checked);
        
        if (path_controller_) {
            path_controller_->setPathEnabled(path_name, checked);
        }
    }
}

void PathVisualizationWidget::onMapCheckBoxToggled(bool checked)
{
    QCheckBox* checkbox = qobject_cast<QCheckBox*>(sender());
    if (checkbox) {
        QString map_name = checkbox->objectName();
        emit mapVisibilityChanged(map_name, checked);
        
        if (map_controller_) {
            map_controller_->setMapEnabled(map_name, checked);
        }
    }
}

void PathVisualizationWidget::onPathColorButtonClicked()
{
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    if (button) {
        QString path_name = button->objectName();
        
        QColor current_color = button->palette().color(QPalette::Button);
        QColor new_color = QColorDialog::getColor(current_color, this, "选择路径颜色");
        
        if (new_color.isValid()) {
            button->setStyleSheet(QString("background-color: %1; border: 1px solid black;")
                                 .arg(new_color.name()));
            
            if (path_controller_) {
                path_controller_->setPathColor(path_name, new_color);
            }
            
            emit pathStyleChanged(path_name);
        }
    }
}

void PathVisualizationWidget::onMapAlphaSliderChanged(int value)
{
    QSlider* slider = qobject_cast<QSlider*>(sender());
    if (slider) {
        QString map_name = slider->objectName();
        float alpha = value / 100.0f;
        
        // 更新标签
        if (map_alpha_labels_.contains(map_name)) {
            map_alpha_labels_[map_name]->setText(QString::number(alpha, 'f', 2));
        }
        
        if (map_controller_) {
            map_controller_->setMapAlpha(map_name, alpha);
        }
        
        emit mapStyleChanged(map_name);
    }
}

void PathVisualizationWidget::onPathWidthChanged(double value)
{
    QSpinBox* spinbox = qobject_cast<QSpinBox*>(sender());
    if (spinbox) {
        QString path_name = spinbox->objectName();
        
        if (path_controller_) {
            path_controller_->setPathLineWidth(path_name, static_cast<float>(value));
        }
        
        emit pathStyleChanged(path_name);
    }
}

void PathVisualizationWidget::onPathStyleChanged(const QString& style)
{
    QComboBox* combo = qobject_cast<QComboBox*>(sender());
    if (combo) {
        QString path_name = combo->objectName();
        
        if (path_controller_) {
            path_controller_->setPathLineStyle(path_name, style);
        }
        
        emit pathStyleChanged(path_name);
    }
}

void PathVisualizationWidget::onViewControlButtonClicked()
{
    QObject* obj = sender();
    if (obj) {
        QString action = obj->objectName();
        emit viewControlRequested(action);
    }
}

void PathVisualizationWidget::updatePathStats()
{
    if (!path_controller_ || !path_controller_->getPathDataModel()) {
        path_stats_label_->setText("路径统计: 无数据");
        return;
    }
    
    auto model = path_controller_->getPathDataModel();
    int total_points = 0;
    int active_paths = 0;
    
    QStringList path_names = {"global_plan", "chennal_path", "coverage_path", 
                             "record_path", "plan", "split_path"};
    
    for (const QString& path_name : path_names) {
        int points = model->getPathPointCount(path_name);
        if (points > 0) {
            total_points += points;
            active_paths++;
        }
    }
    
    path_stats_label_->setText(QString("路径统计: %1 条路径, %2 个点")
                              .arg(active_paths)
                              .arg(total_points));
}

void PathVisualizationWidget::updateMapStats()
{
    if (!map_controller_ || !map_controller_->getMapDataModel()) {
        map_stats_label_->setText("地图统计: 无数据");
        return;
    }
    
    auto model = map_controller_->getMapDataModel();
    int loaded_maps = 0;
    int total_cells = 0;
    
    // 检查主要地图
    if (model->getChennalMap().width > 0) {
        loaded_maps++;
        total_cells += model->getChennalMap().width * model->getChennalMap().height;
    }
    
    if (model->getFreeSpace().width > 0) {
        loaded_maps++;
        total_cells += model->getFreeSpace().width * model->getFreeSpace().height;
    }
    
    if (model->getRiskMap().width > 0) {
        loaded_maps++;
        total_cells += model->getRiskMap().width * model->getRiskMap().height;
    }
    
    map_stats_label_->setText(QString("地图统计: %1 个地图层, %2 个单元格")
                             .arg(loaded_maps)
                             .arg(total_cells));
}
