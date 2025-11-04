#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的日志面板测试程序
测试底部日志布局是否正确显示
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
    from PyQt6.QtCore import Qt, QTimer
    from PyQt6.QtGui import QFont, QPalette, QColor
    
    print("✓ PyQt6 导入成功")
    
    class LogTestWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("日志面板测试 - 底部布局验证")
            
            # 响应式窗口大小
            from PyQt6.QtGui import QGuiApplication
            screen = QGuiApplication.primaryScreen().availableGeometry()
            width = min(1200, int(screen.width() * 0.8))
            height = min(800, int(screen.height() * 0.8))
            self.setGeometry(100, 100, width, height)
            self.setMinimumSize(800, 600)
            
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
                    margin: 5px;
                    padding-top: 10px;
                    color: white;
                }
                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                    padding: 0 5px 0 5px;
                }
            """)
            
            self.setupUI()
            
            # 添加定时器模拟日志更新
            self.log_timer = QTimer()
            self.log_timer.timeout.connect(self.addRandomLog)
            self.log_timer.start(2000)  # 每2秒添加一条日志
            self.log_count = 0
        
        def setupUI(self):
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            
            # 主分割器 (垂直 - 上下布局) - 这是关键！
            main_splitter = QSplitter(Qt.Orientation.Vertical)
            
            # 创建上方分割器 (水平 - 左右布局)
            top_splitter = QSplitter(Qt.Orientation.Horizontal)
            
            # 左侧地图区域
            map_area = self.createMapArea()
            
            # 右侧控制面板
            control_area = self.createControlArea()
            
            # 上方区域：地图 + 控制面板
            top_splitter.addWidget(map_area)
            top_splitter.addWidget(control_area)
            top_splitter.setSizes([600, 400])  # 地图:控制面板 = 3:2
            
            # 底部日志面板 - 重点测试区域
            log_area = self.createLogArea()
            
            # 主布局：上方区域 / 底部日志
            main_splitter.addWidget(top_splitter)
            main_splitter.addWidget(log_area)
            
            # 关键设置：确保日志面板可见
            log_area.setMinimumHeight(200)
            main_splitter.setSizes([500, 250])  # 上方:日志 = 2:1
            
            # 主布局
            layout = QVBoxLayout(central_widget)
            layout.setContentsMargins(5, 5, 5, 5)
            layout.addWidget(main_splitter)
            
            print(f"✓ 布局创建完成")
            print(f"  - 主分割器方向: {'垂直' if main_splitter.orientation() == Qt.Orientation.Vertical else '水平'}")
            print(f"  - 主分割器组件数: {main_splitter.count()}")
            print(f"  - 日志面板最小高度: {log_area.minimumHeight()}px")
        
        def createMapArea(self):
            """创建地图显示区域"""
            map_widget = QWidget()
            map_widget.setStyleSheet("background-color: #1e1e1e; border: 2px solid #4CAF50; border-radius: 4px;")
            layout = QVBoxLayout(map_widget)
            
            title = QLabel("地图显示区域")
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
            title.setStyleSheet("color: #4CAF50; border: none; padding: 20px;")
            
            info = QLabel("(Map Display Area)\n这里显示机器人地图和路径")
            info.setAlignment(Qt.AlignmentFlag.AlignCenter)
            info.setStyleSheet("color: #888888; border: none;")
            
            layout.addWidget(title)
            layout.addWidget(info)
            layout.addStretch()
            
            return map_widget
        
        def createControlArea(self):
            """创建控制面板区域"""
            control_widget = QWidget()
            control_widget.setMinimumWidth(300)
            layout = QVBoxLayout(control_widget)
            
            # 按钮组
            button_group = QGroupBox("控制按钮")
            button_layout = QGridLayout(button_group)
            
            buttons = [
                ("生成FreeSpace", "#4CAF50"), ("生成RiskSpace", "#FF5722"),
                ("建立Chennal Map", "#2196F3"), ("建立BCD", "#FF9800"),
                ("区域记录开始", "#9C27B0"), ("区域记录结束", "#607D8B")
            ]
            
            for i, (text, color) in enumerate(buttons):
                btn = QPushButton(text)
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {color};
                        color: white;
                        border: 2px solid transparent;
                        padding: 8px 12px;
                        border-radius: 4px;
                        font-weight: bold;
                        min-height: 30px;
                    }}
                    QPushButton:hover {{
                        background-color: #f0f0f0;
                        color: {color};
                        border: 2px solid {color};
                    }}
                """)
                btn.clicked.connect(lambda checked, t=text: self.onButtonClicked(t))
                button_layout.addWidget(btn, i // 2, i % 2)
            
            layout.addWidget(button_group)
            layout.addStretch()
            
            return control_widget
        
        def createLogArea(self):
            """创建日志显示区域 - 重点测试"""
            log_widget = QWidget()
            
            # 明显的样式，确保可见
            log_widget.setStyleSheet("""
                QWidget {
                    background-color: #2b2b2b;
                    border: 3px solid #FF5722;  /* 红色边框，便于识别 */
                    border-radius: 6px;
                }
            """)
            
            layout = QVBoxLayout(log_widget)
            layout.setContentsMargins(8, 8, 8, 8)
            
            # 日志控制栏
            control_group = QGroupBox("日志控制 (Log Control) - 底部面板测试")
            control_layout = QHBoxLayout(control_group)
            
            # 过滤器
            filter_label = QLabel("过滤:")
            filter_combo = QComboBox()
            filter_combo.addItems(["全部", "信息", "警告", "错误", "成功"])
            
            # 按钮
            clear_btn = QPushButton("清空")
            clear_btn.clicked.connect(self.clearLog)
            test_btn = QPushButton("测试日志")
            test_btn.clicked.connect(self.addRandomLog)
            
            for btn in [clear_btn, test_btn]:
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #4CAF50;
                        color: white;
                        border: none;
                        padding: 4px 8px;
                        border-radius: 3px;
                        font-weight: bold;
                        min-height: 24px;
                    }
                    QPushButton:hover {
                        background-color: #45a049;
                    }
                """)
            
            control_layout.addWidget(filter_label)
            control_layout.addWidget(filter_combo)
            control_layout.addStretch()
            control_layout.addWidget(clear_btn)
            control_layout.addWidget(test_btn)
            
            # 日志显示区域
            self.log_display = QTextEdit()
            self.log_display.setReadOnly(True)
            self.log_display.setMinimumHeight(120)
            self.log_display.setStyleSheet("""
                QTextEdit {
                    background-color: #1e1e1e;
                    color: #ffffff;
                    border: 1px solid #555555;
                    border-radius: 4px;
                    font-family: 'Consolas', 'Monaco', monospace;
                    font-size: 11px;
                    padding: 4px;
                }
            """)
            
            # 添加初始日志
            self.addInitialLogs()
            
            layout.addWidget(control_group)
            layout.addWidget(self.log_display)
            
            return log_widget
        
        def addInitialLogs(self):
            """添加初始日志"""
            logs = [
                '<span style="color: #888888;">[12:34:56.001]</span> <span style="color: #2196F3; font-weight: bold;">[信息]</span> <span style="color: #ffffff;">日志系统已启动 - 底部面板测试</span>',
                '<span style="color: #888888;">[12:34:56.002]</span> <span style="color: #4CAF50; font-weight: bold;">[成功]</span> <span style="color: #ffffff;">Qt界面初始化完成</span>',
                '<span style="color: #888888;">[12:34:56.003]</span> <span style="color: #9C27B0; font-weight: bold;">[订阅]</span> <span style="color: #ffffff;">等待ROS2节点连接...</span>',
                '<span style="color: #888888;">[12:34:56.004]</span> <span style="color: #607D8B; font-weight: bold;">[调试]</span> <span style="color: #ffffff;">界面组件加载完成</span>',
                '<span style="color: #888888;">[12:34:56.005]</span> <span style="color: #FF9800; font-weight: bold;">[警告]</span> <span style="color: #ffffff;">这是底部日志面板测试</span>',
            ]
            
            for log in logs:
                self.log_display.append(log)
        
        def addRandomLog(self):
            """添加随机日志"""
            import datetime
            self.log_count += 1
            
            log_types = [
                ("信息", "#2196F3", f"测试日志消息 #{self.log_count}"),
                ("成功", "#4CAF50", f"操作成功完成 #{self.log_count}"),
                ("警告", "#FF9800", f"注意事项 #{self.log_count}"),
                ("错误", "#F44336", f"测试错误消息 #{self.log_count}"),
                ("订阅", "#9C27B0", f"订阅话题: /test_topic_{self.log_count}"),
                ("服务", "#00BCD4", f"服务调用: /test_service_{self.log_count}"),
            ]
            
            import random
            log_type, color, message = random.choice(log_types)
            timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            log_entry = f'<span style="color: #888888;">[{timestamp}]</span> <span style="color: {color}; font-weight: bold;">[{log_type}]</span> <span style="color: #ffffff;">{message}</span>'
            self.log_display.append(log_entry)
            
            # 自动滚动到底部
            scrollbar = self.log_display.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
        
        def clearLog(self):
            """清空日志"""
            self.log_display.clear()
            self.addInitialLogs()
            self.log_count = 0
        
        def onButtonClicked(self, button_text):
            """按钮点击事件"""
            import datetime
            timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            log_entry = f'<span style="color: #888888;">[{timestamp}]</span> <span style="color: #00BCD4; font-weight: bold;">[服务]</span> <span style="color: #ffffff;">按钮点击: {button_text}</span>'
            self.log_display.append(log_entry)
            
            # 自动滚动到底部
            scrollbar = self.log_display.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            
            print(f"按钮点击: {button_text}")
    
    def main():
        app = QApplication(sys.argv)
        
        # 设置应用程序信息
        app.setApplicationName("日志面板测试")
        app.setApplicationVersion("1.0")
        app.setOrganizationName("Lawer Robotics")
        
        # 设置深色主题
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(43, 43, 43))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.Base, QColor(30, 30, 30))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(0, 0, 0))
        palette.setColor(QPalette.ColorRole.ToolTipText, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.Text, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.BrightText, QColor(255, 0, 0))
        palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor(0, 0, 0))
        app.setPalette(palette)
        
        # 创建并显示窗口
        window = LogTestWindow()
        window.show()
        
        print("✓ 日志面板测试程序启动")
        print("✓ 请检查窗口底部是否显示红色边框的日志面板")
        print("✓ 日志面板应该包含控制栏和日志显示区域")
        print("✓ 每2秒会自动添加一条测试日志")
        print("✓ 点击按钮也会添加日志条目")
        
        return app.exec()

except ImportError as e:
    print(f"✗ PyQt6 导入失败: {e}")
    print("请安装PyQt6: pip install PyQt6")
    sys.exit(1)

if __name__ == "__main__":
    sys.exit(main())
