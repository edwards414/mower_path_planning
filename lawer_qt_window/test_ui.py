#!/usr/bin/env python3
"""
简单的测试脚本，用于验证Qt界面是否正常工作
这个脚本不需要完整的ROS2环境，只测试UI组件
"""

import sys
import os

# 添加Qt6路径 (如果需要)
qt6_path = "/usr/lib/x86_64-linux-gnu/qt6/bin"
if os.path.exists(qt6_path):
    os.environ["PATH"] = qt6_path + ":" + os.environ.get("PATH", "")

try:
    from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget
    from PyQt6.QtWidgets import QPushButton, QLabel, QGroupBox, QGridLayout, QSplitter
    from PyQt6.QtWidgets import QScrollArea, QFrame, QTextEdit, QComboBox, QCheckBox
    from PyQt6.QtCore import Qt
    from PyQt6.QtGui import QFont, QPalette, QColor
    
    print("✓ PyQt6 导入成功")
    
    class TestMainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Lawer Qt Window - 响应式UI测试")
            
            # 响应式窗口大小
            from PyQt6.QtGui import QGuiApplication
            screen = QGuiApplication.primaryScreen().availableGeometry()
            width = min(1200, int(screen.width() * 0.8))
            height = min(800, int(screen.height() * 0.8))
            self.setGeometry(100, 100, width, height)
            self.setMinimumSize(600, 400)
            
            # 设置深色主题
            self.setStyleSheet("""
                QMainWindow {
                    background-color: #2b2b2b;
                    color: white;
                }
                QGroupBox {
                    font-weight: bold;
                    border: 2px solid #555555;
                    border-radius: 5px;
                    margin-top: 1ex;
                    padding-top: 10px;
                }
                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                    padding: 0 5px 0 5px;
                }
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 10px 16px;
                    border-radius: 6px;
                    font-weight: bold;
                    min-height: 32px;
                }
                QPushButton:hover {
                    background-color: #5CBF60;
                }
                QPushButton:pressed {
                    background-color: #3E8E41;
                }
            """)
            
            self.setupUI()
        
        def setupUI(self):
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            
            # 主分割器 (垂直 - 上下布局)
            main_splitter = QSplitter(Qt.Orientation.Vertical)
            
            # 创建上方分割器 (水平 - 左右布局)
            top_splitter = QSplitter(Qt.Orientation.Horizontal)
            
            # 左侧地图区域
            map_area = QWidget()
            map_area.setStyleSheet("background-color: #2b2b2b; border: 1px solid #555555;")
            map_layout = QVBoxLayout(map_area)
            map_label = QLabel("地图显示区域\n(Map Display Area)")
            map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            map_label.setFont(QFont("Arial", 16))
            map_label.setStyleSheet("color: white; border: none;")
            map_layout.addWidget(map_label)
            
            # 右侧面板 (垂直分割器)
            right_splitter = QSplitter(Qt.Orientation.Vertical)
            
            # 控制面板和路径可视化
            control_panel = self.createControlPanel()
            path_viz = self.createPathVisualization()
            
            right_splitter.addWidget(control_panel)
            right_splitter.addWidget(path_viz)
            right_splitter.setSizes([300, 200])  # 控制面板:路径可视化 = 3:2
            
            # 上方区域：地图 + 右侧面板
            top_splitter.addWidget(map_area)
            top_splitter.addWidget(right_splitter)
            top_splitter.setSizes([600, 400])  # 地图:右侧面板 = 3:2
            
            # 底部日志面板
            log_panel = self.createLogPanel()
            
            # 主布局：上方区域 / 底部日志
            main_splitter.addWidget(top_splitter)
            main_splitter.addWidget(log_panel)
            main_splitter.setSizes([500, 200])  # 上方:日志 = 5:2
            
            # 主布局
            layout = QHBoxLayout(central_widget)
            layout.addWidget(main_splitter)
        
        def createControlPanel(self):
            # 创建滚动区域
            scroll_area = QScrollArea()
            scroll_area.setWidgetResizable(True)
            scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
            scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
            scroll_area.setFrameShape(QFrame.Shape.NoFrame)
            
            widget = QWidget()
            widget.setMinimumWidth(250)
            layout = QVBoxLayout(widget)
            
            # 地图管理组
            map_group = QGroupBox("地图管理 (Map Management)")
            map_layout = QGridLayout(map_group)
            
            buttons = [
                ("生成FreeSpace", "#4CAF50"),
                ("生成RiskSpace", "#FF9800"),
                ("建立Chennal Map", "#2196F3"),
                ("建立BCD", "#9C27B0"),
            ]
            
            for i, (text, color) in enumerate(buttons):
                btn = QPushButton(text)
                btn.setStyleSheet(f"QPushButton {{ background-color: {color}; }}")
                btn.clicked.connect(lambda checked, t=text: self.onButtonClicked(t))
                map_layout.addWidget(btn, i // 2, i % 2)
            
            layout.addWidget(map_group)
            
            # 路径记录组
            path_group = QGroupBox("路径记录 (Path Recording)")
            path_layout = QVBoxLayout(path_group)
            
            path_buttons = [
                ["区域记录开始", "区域记录结束", "保存区域", "加载区域"],
                ["Risk区域开始", "Risk区域结束", "保存Risk区域", "获取记录信息"],
                ["Chennal记录开始", "Chennal记录结束", "获取Chennal列表"]
            ]
            
            colors = ["#4CAF50", "#F44336", "#2196F3", "#FF9800"]
            
            for row_buttons in path_buttons:
                row_layout = QHBoxLayout()
                for i, text in enumerate(row_buttons):
                    btn = QPushButton(text)
                    color = colors[i % len(colors)]
                    btn.setStyleSheet(f"QPushButton {{ background-color: {color}; }}")
                    btn.clicked.connect(lambda checked, t=text: self.onButtonClicked(t))
                    row_layout.addWidget(btn)
                path_layout.addLayout(row_layout)
            
            layout.addWidget(path_group)
            
            # 覆盖路径规划组
            coverage_group = QGroupBox("覆盖路径规划 (Coverage Planning)")
            coverage_layout = QHBoxLayout(coverage_group)
            
            coverage_buttons = [
                ("生成覆盖路径", "#4CAF50"),
                ("发布航点", "#2196F3"),
                ("取消导航", "#F44336")
            ]
            
            for text, color in coverage_buttons:
                btn = QPushButton(text)
                btn.setStyleSheet(f"QPushButton {{ background-color: {color}; }}")
                btn.clicked.connect(lambda checked, t=text: self.onButtonClicked(t))
                coverage_layout.addWidget(btn)
            
            layout.addWidget(coverage_group)
            
            # 状态信息
            status_group = QGroupBox("状态信息 (Status)")
            status_layout = QVBoxLayout(status_group)
            
            self.status_label = QLabel("就绪 - UI测试模式")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            status_layout.addWidget(self.status_label)
            
            layout.addWidget(status_group)
            layout.addStretch()
            
            # 设置滚动内容
            scroll_area.setWidget(widget)
            
            return scroll_area
        
        def createPathVisualization(self):
            """创建路径可视化面板"""
            path_widget = QWidget()
            path_widget.setMinimumWidth(250)
            layout = QVBoxLayout(path_widget)
            
            # 路径控制组
            path_group = QGroupBox("路径显示控制 (Path Control)")
            path_layout = QVBoxLayout(path_group)
            
            # 路径可见性控制
            path_checkboxes = []
            path_types = ["覆盖路径", "导航路径", "局部路径", "机器人轨迹"]
            for path_type in path_types:
                checkbox = QCheckBox(path_type)
                checkbox.setChecked(True)
                checkbox.setStyleSheet("""
                    QCheckBox {
                        color: white;
                        font-size: 11px;
                        spacing: 8px;
                    }
                    QCheckBox::indicator {
                        width: 16px;
                        height: 16px;
                    }
                    QCheckBox::indicator:checked {
                        background-color: #4CAF50;
                        border: 2px solid #4CAF50;
                        border-radius: 3px;
                    }
                    QCheckBox::indicator:unchecked {
                        background-color: transparent;
                        border: 2px solid #888888;
                        border-radius: 3px;
                    }
                """)
                path_checkboxes.append(checkbox)
                path_layout.addWidget(checkbox)
            
            # 地图控制组
            map_group = QGroupBox("地图显示控制 (Map Control)")
            map_layout = QVBoxLayout(map_group)
            
            # 地图可见性控制
            map_checkboxes = []
            map_types = ["自由空间", "风险地图", "通道地图", "代价地图"]
            for map_type in map_types:
                checkbox = QCheckBox(map_type)
                checkbox.setChecked(True)
                checkbox.setStyleSheet("""
                    QCheckBox {
                        color: white;
                        font-size: 11px;
                        spacing: 8px;
                    }
                    QCheckBox::indicator {
                        width: 16px;
                        height: 16px;
                    }
                    QCheckBox::indicator:checked {
                        background-color: #2196F3;
                        border: 2px solid #2196F3;
                        border-radius: 3px;
                    }
                    QCheckBox::indicator:unchecked {
                        background-color: transparent;
                        border: 2px solid #888888;
                        border-radius: 3px;
                    }
                """)
                map_checkboxes.append(checkbox)
                map_layout.addWidget(checkbox)
            
            layout.addWidget(path_group)
            layout.addWidget(map_group)
            layout.addStretch()
            
            return path_widget
        
        def createLogPanel(self):
            """创建日志面板"""
            log_widget = QWidget()
            log_widget.setMinimumWidth(250)
            layout = QVBoxLayout(log_widget)
            
            # 日志控制组
            control_group = QGroupBox("日志控制 (Log Control)")
            control_layout = QHBoxLayout(control_group)
            
            # 过滤器和按钮
            filter_label = QLabel("过滤:")
            filter_combo = QComboBox()
            filter_combo.addItems(["全部", "信息", "警告", "错误", "成功", "话题订阅", "服务调用"])
            
            clear_btn = QPushButton("清空")
            save_btn = QPushButton("保存")
            
            # 设置按钮样式 (带反色效果)
            for btn, color in [(clear_btn, "#FF5722"), (save_btn, "#4CAF50")]:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {color};
                        color: white;
                        border: 2px solid transparent;
                        padding: 4px 8px;
                        border-radius: 4px;
                        font-weight: bold;
                        font-size: 10px;
                        min-height: 20px;
                    }}
                    QPushButton:hover {{
                        background-color: #f0f0f0;
                        color: {color};
                        border: 2px solid {color};
                        transform: translateY(-1px);
                    }}
                    QPushButton:pressed {{
                        background-color: #cccccc;
                        transform: translateY(1px);
                    }}
                """)
            
            control_layout.addWidget(filter_label)
            control_layout.addWidget(filter_combo)
            control_layout.addStretch()
            control_layout.addWidget(clear_btn)
            control_layout.addWidget(save_btn)
            
            # 日志显示区域
            log_group = QGroupBox("系统日志 (System Logs)")
            log_layout = QVBoxLayout(log_group)
            
            from PyQt6.QtWidgets import QTextEdit
            log_display = QTextEdit()
            log_display.setReadOnly(True)
            log_display.setMinimumHeight(150)
            log_display.setStyleSheet("""
                QTextEdit {
                    background-color: #1e1e1e;
                    color: #ffffff;
                    border: 1px solid #555555;
                    border-radius: 4px;
                    font-family: 'Consolas', 'Monaco', monospace;
                    font-size: 10px;
                    padding: 4px;
                }
            """)
            
            # 添加示例日志
            sample_logs = [
                '<span style="color: #888888;">[12:34:56.789]</span> <span style="color: #9C27B0; font-weight: bold;">[订阅]</span> <span style="color: #ffffff;">订阅话题: /chennal_map [nav_msgs/msg/OccupancyGrid]</span>',
                '<span style="color: #888888;">[12:34:56.790]</span> <span style="color: #9C27B0; font-weight: bold;">[订阅]</span> <span style="color: #ffffff;">订阅话题: /free_space [nav_msgs/msg/OccupancyGrid]</span>',
                '<span style="color: #888888;">[12:34:56.791]</span> <span style="color: #00BCD4; font-weight: bold;">[服务]</span> <span style="color: #ffffff;">服务调用: /create_free_space [std_srvs/srv/Trigger] - 成功</span>',
                '<span style="color: #888888;">[12:34:56.792]</span> <span style="color: #4CAF50; font-weight: bold;">[成功]</span> <span style="color: #ffffff;">地图控制器初始化成功，已订阅所有地图话题</span>',
                '<span style="color: #888888;">[12:34:56.793]</span> <span style="color: #607D8B; font-weight: bold;">[调试]</span> <span style="color: #ffffff;">收到消息: /chennal_map - 收到地图数据</span>',
            ]
            
            for log in sample_logs:
                log_display.append(log)
            
            log_layout.addWidget(log_display)
            
            layout.addWidget(control_group)
            layout.addWidget(log_group)
            
            return log_widget
        
        def onButtonClicked(self, button_text):
            self.status_label.setText(f"按钮点击: {button_text}")
            print(f"按钮点击: {button_text}")
            
            # 模拟添加日志 (如果有日志显示的话)
            print(f"[LOG] 服务调用: {button_text} - 成功")
    
    def main():
        app = QApplication(sys.argv)
        
        # 设置应用程序信息
        app.setApplicationName("Lawer Qt Window UI Test")
        app.setApplicationVersion("1.0.0")
        
        print("启动UI测试窗口...")
        
        window = TestMainWindow()
        window.show()
        
        print("UI测试窗口已显示")
        print("测试说明:")
        print("1. 检查窗口是否正常显示")
        print("2. 检查按钮布局是否正确")
        print("3. 点击按钮测试响应")
        print("4. 检查深色主题是否生效")
        
        return app.exec()

except ImportError as e:
    print(f"✗ PyQt6 导入失败: {e}")
    print("请安装PyQt6: pip install PyQt6")
    sys.exit(1)

if __name__ == "__main__":
    sys.exit(main())
