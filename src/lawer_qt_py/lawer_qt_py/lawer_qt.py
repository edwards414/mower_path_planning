#!/usr/bin/env python3
import sys
from uuid import uuid4
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import (
    Parameter as ParameterMsg,
    ParameterType,
    ParameterValue,
)
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QGroupBox, 
                             QScrollArea, QLabel, QSpinBox, QDoubleSpinBox,
                             QCheckBox, QFormLayout)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont
from datetime import datetime
from mower_interface.srv import ZoneExecPath


COVERAGE_PARAMETER_DEFAULTS = {
    'strip_width_m': 0.2,
    'waypoint_spacing_m': 0.1,
    'unknown_as_obstacle': True,
    'inflate_radius_m': 0.55,
}


class ServiceCallThread(QThread):
    """异步调用ROS2服务的线程"""
    result_signal = pyqtSignal(str, bool, str)  # service_name, success, message
    
    def __init__(self, service_name, service_type=Trigger, request_data=None):
        super().__init__()
        self.service_name = service_name
        self.service_type = service_type
        self.request_data = request_data
        
    def run(self):
        node = None
        executor = None
        try:
            node = Node(f'lawer_qt_service_client_{uuid4().hex}')
            client = node.create_client(self.service_type, self.service_name)
            
            # 等待服务可用
            if not client.wait_for_service(timeout_sec=3.0):
                self.result_signal.emit(
                    self.service_name, 
                    False, 
                    f"服务 {self.service_name} 不可用"
                )
                return
            
            # 创建请求
            if self.service_type == Trigger:
                request = Trigger.Request()
            elif self.service_type == ZoneExecPath:
                request = ZoneExecPath.Request()
                if self.request_data:
                    request.zone_id = self.request_data.get('zone_id', 0)
            
            # 调用服务
            future = client.call_async(request)
            
            # 等待响应
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.spin_until_future_complete(future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                self.result_signal.emit(
                    self.service_name,
                    response.success,
                    response.message if hasattr(response, 'message') and response.message else "服务调用成功"
                )
            else:
                self.result_signal.emit(
                    self.service_name,
                    False,
                    "服务调用超时或失败"
                )
                
        except Exception as e:
            self.result_signal.emit(
                self.service_name,
                False,
                f"错误: {str(e)}"
            )
        finally:
            if executor is not None:
                if node is not None:
                    executor.remove_node(node)
                executor.shutdown()
            if node is not None:
                node.destroy_node()


class ParameterSetThread(QThread):
    """异步设置ROS2参数的线程"""
    result_signal = pyqtSignal(str, bool, str)  # service_name, success, message

    def __init__(self, service_name, parameters):
        super().__init__()
        self.service_name = service_name
        self.parameters = parameters

    def run(self):
        node = None
        executor = None
        try:
            node = Node(f'lawer_qt_param_client_{uuid4().hex}')
            client = node.create_client(SetParameters, self.service_name)

            if not client.wait_for_service(timeout_sec=3.0):
                self.result_signal.emit(
                    self.service_name,
                    False,
                    f"参数服务 {self.service_name} 不可用"
                )
                return

            request = SetParameters.Request()
            for param in self.parameters:
                request.parameters.append(self.create_parameter_msg(param))

            future = client.call_async(request)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.spin_until_future_complete(future, timeout_sec=5.0)

            if future.result() is None:
                self.result_signal.emit(
                    self.service_name,
                    False,
                    "参数设置超时或失败"
                )
                return

            response = future.result()
            failed = []
            for param, result in zip(self.parameters, response.results):
                if not result.successful:
                    reason = result.reason or "被节点拒绝"
                    failed.append(f"{param['name']}: {reason}")

            if failed:
                self.result_signal.emit(
                    self.service_name,
                    False,
                    "; ".join(failed)
                )
                return

            names = ", ".join(param['name'] for param in self.parameters)
            self.result_signal.emit(
                self.service_name,
                True,
                f"已设置参数: {names}"
            )

        except Exception as e:
            self.result_signal.emit(
                self.service_name,
                False,
                f"错误: {str(e)}"
            )
        finally:
            if executor is not None:
                if node is not None:
                    executor.remove_node(node)
                executor.shutdown()
            if node is not None:
                node.destroy_node()

    @staticmethod
    def create_parameter_msg(param):
        value = ParameterValue()
        if param['type'] == 'double':
            value.type = ParameterType.PARAMETER_DOUBLE
            value.double_value = float(param['value'])
        elif param['type'] == 'bool':
            value.type = ParameterType.PARAMETER_BOOL
            value.bool_value = bool(param['value'])
        else:
            raise ValueError(f"Unsupported parameter type: {param['type']}")

        return ParameterMsg(name=param['name'], value=value)


class LawerQtWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 初始化ROS2
        rclpy.init()
        self.node = Node('lawer_qt_node')
        self.parameter_threads = []
        
        self.setWindowTitle('ROS2 服务控制面板')
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建主窗口部件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 主布局
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        
        # 左侧：按钮区域
        button_scroll = QScrollArea()
        button_scroll.setWidgetResizable(True)
        button_widget = QWidget()
        button_layout = QVBoxLayout()
        button_widget.setLayout(button_layout)
        button_scroll.setWidget(button_widget)
        
        # 右侧：日志区域
        log_widget = QWidget()
        log_layout = QVBoxLayout()
        log_widget.setLayout(log_layout)
        
        log_label = QLabel('日志输出')
        log_label.setFont(QFont('Arial', 12, QFont.Bold))
        log_layout.addWidget(log_label)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont('Courier', 10))
        log_layout.addWidget(self.log_text)
        
        # 清除日志按钮
        clear_btn = QPushButton('清除日志')
        clear_btn.clicked.connect(self.clear_log)
        log_layout.addWidget(clear_btn)
        
        # 添加到主布局
        main_layout.addWidget(button_scroll, 1)
        main_layout.addWidget(log_widget, 1)
        
        # 创建按钮组
        self.create_button_groups(button_layout)
        
        # 添加弹性空间
        button_layout.addStretch()
        
        self.log_message("系统启动", "INFO")
        
    def create_button_groups(self, layout):
        """创建所有按钮组"""
        
        # Path Record 组
        path_record_group = self.create_group_box("Path Record")
        path_record_layout = QVBoxLayout()
        path_record_group.setLayout(path_record_layout)
        
        path_record_services = [
            ("記錄區域 起始點", "/record_zone_start"),
            ("記錄區域 結束點", "/record_zone_end"),
            ("記錄Risk Zone 起始點", "/risk_zone_start"),
            ("記錄Risk Zone 結束點", "/risk_zone_end"),
            ("記錄區域 Save", "/save_zone_list"),
            ("記錄區域 Risk Save", "/risk_zone_save"),
            ("記錄區域 Load", "/load_zone_list"),
            ("Path Record Info Test", "/get_record_zone_info"),
            ("記錄Chennal Start", "/chennal_record_start"),
            ("記錄Chennal End", "/chennal_record_end"),
            ("讀取Chennal Path List", "/get_chennal_path_list"),
        ]
        
        for btn_text, service_name in path_record_services:
            btn = self.create_service_button(btn_text, service_name)
            path_record_layout.addWidget(btn)
        
        layout.addWidget(path_record_group)
        
        # Map Manage 组
        map_manage_group = self.create_group_box("Map Manage")
        map_manage_layout = QVBoxLayout()
        map_manage_group.setLayout(map_manage_layout)
        
        map_manage_services = [
            ("生成多個Zone的Freespace", "/create_free_space"),
            ("生成多個Zone的Riskspace", "/create_risk_map"),
            ("建立Chennal Map", "/create_chennal_map"),
            ("取得Record Zone List", "/get_record_zone_list_srv"),
        ]
        
        for btn_text, service_name in map_manage_services:
            btn = self.create_service_button(btn_text, service_name)
            map_manage_layout.addWidget(btn)
        
        layout.addWidget(map_manage_group)
        
        # Boustrophedon Coverage 组
        coverage_group = self.create_group_box("Boustrophedon Coverage")
        coverage_layout = QVBoxLayout()
        coverage_group.setLayout(coverage_layout)

        self.create_coverage_parameter_controls(coverage_layout)
        
        coverage_services = [
            ("生成Coverage Path", "/generate_coverage_path"),
            ("Waypoint Pub", "/waypoint_pub"),
            ("取消Nav2", "/cencel_nav2"),
        ]
        
        for btn_text, service_name in coverage_services:
            btn = self.create_service_button(btn_text, service_name)
            coverage_layout.addWidget(btn)
        
        # 添加 Zone Exec Path 输入框和按钮
        zone_exec_widget = QWidget()
        zone_exec_layout = QHBoxLayout()
        zone_exec_widget.setLayout(zone_exec_layout)
        
        zone_label = QLabel("Zone ID:")
        zone_label.setFont(QFont('Arial', 10))
        zone_exec_layout.addWidget(zone_label)
        
        self.zone_id_spinbox = QSpinBox()
        self.zone_id_spinbox.setMinimum(0)
        self.zone_id_spinbox.setMaximum(999)
        self.zone_id_spinbox.setValue(1)
        self.zone_id_spinbox.setFont(QFont('Arial', 10))
        zone_exec_layout.addWidget(self.zone_id_spinbox)
        
        zone_exec_btn = QPushButton("執行Zone Path")
        zone_exec_btn.setFont(QFont('Arial', 10))
        zone_exec_btn.setMinimumHeight(40)
        zone_exec_btn.clicked.connect(self.call_zone_exec_path)
        zone_exec_layout.addWidget(zone_exec_btn)
        
        coverage_layout.addWidget(zone_exec_widget)
        
        layout.addWidget(coverage_group)

    def create_coverage_parameter_controls(self, layout):
        """创建覆盖路径参数控制"""
        title_label = QLabel("Coverage Parameters")
        title_label.setFont(QFont('Arial', 10, QFont.Bold))
        layout.addWidget(title_label)

        param_widget = QWidget()
        param_layout = QFormLayout()
        param_widget.setLayout(param_layout)

        self.strip_width_spinbox = self.create_double_spinbox(
            COVERAGE_PARAMETER_DEFAULTS['strip_width_m'],
            minimum=0.01,
            maximum=5.0,
            step=0.05,
        )
        param_layout.addRow("割幅 strip_width_m (m):",
                            self.strip_width_spinbox)

        self.waypoint_spacing_spinbox = self.create_double_spinbox(
            COVERAGE_PARAMETER_DEFAULTS['waypoint_spacing_m'],
            minimum=0.01,
            maximum=2.0,
            step=0.05,
        )
        param_layout.addRow("Waypoint 間距 (m):",
                            self.waypoint_spacing_spinbox)

        self.inflate_radius_spinbox = self.create_double_spinbox(
            COVERAGE_PARAMETER_DEFAULTS['inflate_radius_m'],
            minimum=0.0,
            maximum=3.0,
            step=0.05,
        )
        param_layout.addRow("安全膨脹半徑 inflate_radius_m (m):",
                            self.inflate_radius_spinbox)

        self.unknown_as_obstacle_checkbox = QCheckBox("未知區域視為障礙")
        self.unknown_as_obstacle_checkbox.setChecked(
            COVERAGE_PARAMETER_DEFAULTS['unknown_as_obstacle'])
        self.unknown_as_obstacle_checkbox.setFont(QFont('Arial', 10))
        param_layout.addRow("unknown_as_obstacle:",
                            self.unknown_as_obstacle_checkbox)

        layout.addWidget(param_widget)

        button_widget = QWidget()
        button_layout = QHBoxLayout()
        button_widget.setLayout(button_layout)

        apply_btn = QPushButton("套用覆蓋參數")
        apply_btn.setFont(QFont('Arial', 10))
        apply_btn.setMinimumHeight(36)
        apply_btn.clicked.connect(self.apply_coverage_parameters)
        button_layout.addWidget(apply_btn)

        reset_btn = QPushButton("還原預設")
        reset_btn.setFont(QFont('Arial', 10))
        reset_btn.setMinimumHeight(36)
        reset_btn.clicked.connect(self.reset_coverage_parameters)
        button_layout.addWidget(reset_btn)

        layout.addWidget(button_widget)

    def create_double_spinbox(self, value, minimum, maximum, step):
        """创建浮点参数输入框"""
        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(minimum)
        spinbox.setMaximum(maximum)
        spinbox.setSingleStep(step)
        spinbox.setDecimals(2)
        spinbox.setValue(value)
        spinbox.setFont(QFont('Arial', 10))
        return spinbox
    
    def create_group_box(self, title):
        """创建分组框"""
        group_box = QGroupBox(title)
        group_box.setFont(QFont('Arial', 11, QFont.Bold))
        return group_box
    
    def create_service_button(self, text, service_name):
        """创建服务调用按钮"""
        btn = QPushButton(text)
        btn.setMinimumHeight(40)
        btn.setFont(QFont('Arial', 10))
        btn.clicked.connect(lambda: self.call_service(service_name, text))
        return btn
    
    def call_service(self, service_name, button_text):
        """调用ROS2服务"""
        self.log_message(f"正在调用服务: {service_name} ({button_text})", "INFO")
        
        # 创建并启动服务调用线程
        self.service_thread = ServiceCallThread(service_name)
        self.service_thread.result_signal.connect(self.on_service_result)
        self.service_thread.start()

    def apply_coverage_parameters(self):
        """套用覆盖路径相关参数"""
        coverage_params = [
            {
                'name': 'strip_width_m',
                'type': 'double',
                'value': self.strip_width_spinbox.value(),
            },
            {
                'name': 'waypoint_spacing_m',
                'type': 'double',
                'value': self.waypoint_spacing_spinbox.value(),
            },
            {
                'name': 'unknown_as_obstacle',
                'type': 'bool',
                'value': self.unknown_as_obstacle_checkbox.isChecked(),
            },
        ]
        map_params = [
            {
                'name': 'inflate_radius_m',
                'type': 'double',
                'value': self.inflate_radius_spinbox.value(),
            },
        ]

        targets = [
            ('/boustrophedon_coverage/set_parameters', coverage_params),
            ('/map_manage/set_parameters', map_params),
        ]

        self.log_message("正在套用覆盖参数", "INFO")
        for service_name, params in targets:
            thread = ParameterSetThread(service_name, params)
            thread.result_signal.connect(self.on_parameter_result)
            thread.finished.connect(
                lambda thread=thread: self.cleanup_parameter_thread(thread)
            )
            self.parameter_threads.append(thread)
            thread.start()

    def reset_coverage_parameters(self):
        """还原覆盖路径参数输入值"""
        self.strip_width_spinbox.setValue(
            COVERAGE_PARAMETER_DEFAULTS['strip_width_m'])
        self.waypoint_spacing_spinbox.setValue(
            COVERAGE_PARAMETER_DEFAULTS['waypoint_spacing_m'])
        self.inflate_radius_spinbox.setValue(
            COVERAGE_PARAMETER_DEFAULTS['inflate_radius_m'])
        self.unknown_as_obstacle_checkbox.setChecked(
            COVERAGE_PARAMETER_DEFAULTS['unknown_as_obstacle'])
        self.log_message("覆盖参数输入值已还原为当前程式预设", "INFO")

    def cleanup_parameter_thread(self, thread):
        """清理已完成的参数设置线程"""
        if thread in self.parameter_threads:
            self.parameter_threads.remove(thread)

    def on_parameter_result(self, service_name, success, message):
        """处理参数设置结果"""
        if success:
            self.log_message(
                f"参数服务 {service_name} 设置成功: {message}",
                "SUCCESS"
            )
        else:
            self.log_message(
                f"参数服务 {service_name} 设置失败: {message}",
                "ERROR"
            )
    
    def call_zone_exec_path(self):
        """调用 zone_exec_path 服务"""
        zone_id = self.zone_id_spinbox.value()
        service_name = "/zone_exec_path"
        
        self.log_message(
            f"正在调用服务: {service_name} (Zone ID: {zone_id})", 
            "INFO"
        )
        
        # 创建并启动服务调用线程
        self.service_thread = ServiceCallThread(
            service_name, 
            service_type=ZoneExecPath,
            request_data={'zone_id': zone_id}
        )
        self.service_thread.result_signal.connect(self.on_service_result)
        self.service_thread.start()
    
    def on_service_result(self, service_name, success, message):
        """处理服务调用结果"""
        if success:
            self.log_message(
                f"服务 {service_name} 调用成功: {message}", 
                "SUCCESS"
            )
        else:
            self.log_message(
                f"服务 {service_name} 调用失败: {message}", 
                "ERROR"
            )
    
    def log_message(self, message, level="INFO"):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # 根据级别设置颜色
        color_map = {
            "INFO": "#2196F3",      # 蓝色
            "SUCCESS": "#4CAF50",   # 绿色
            "ERROR": "#F44336",     # 红色
            "WARNING": "#FF9800"    # 橙色
        }
        
        color = color_map.get(level, "#000000")
        
        log_entry = f'<span style="color: {color};">[{timestamp}] [{level}] {message}</span><br>'
        self.log_text.append(log_entry)
        
        # 自动滚动到底部
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def clear_log(self):
        """清除日志"""
        self.log_text.clear()
        self.log_message("日志已清除", "INFO")
    
    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    window = LawerQtWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

