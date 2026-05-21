#!/usr/bin/env python3
import json
import sys
from threading import Lock
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
                             QCheckBox, QFormLayout, QComboBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont
from datetime import datetime
from mower_interface.srv import ChennalPathList, GetZoneList, ZoneExecPath


_RCLPY_INIT_LOCK = Lock()


def ensure_rclpy_initialized():
    """Ensure the shared rclpy context exists before any worker creates nodes."""
    with _RCLPY_INIT_LOCK:
        if not rclpy.ok():
            rclpy.init(args=None)


COVERAGE_PARAMETER_DEFAULTS = {
    'strip_width_m': 0.8,
    'waypoint_spacing_m': 0.2,
    'zigzag_angle_deg': 0.0,
    'unknown_as_obstacle': True,
    'inflate_radius_m': 0.55,
    'coverage_pattern': 'zigzag',
}

DOCKING_STATUS_SERVICE = "/mower_docking_status"
DOCKING_COMMAND_SERVICES = {
    "/mower_dock_home",
    "/mower_undock",
    "/mower_cancel_docking",
    "/record_home_dock_pose",
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
            ensure_rclpy_initialized()
            node = Node(f'mower_qt_service_client_{uuid4().hex}')
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
            request = self.service_type.Request()
            if self.request_data:
                for field_name, value in self.request_data.items():
                    if hasattr(request, field_name):
                        setattr(request, field_name, value)
            
            # 调用服务
            future = client.call_async(request)
            
            # 等待响应
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.spin_until_future_complete(future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                message = (
                    response.message
                    if hasattr(response, 'message') and response.message
                    else "服务调用成功"
                )
                if hasattr(response, 'zone_list'):
                    count = len(response.zone_list.markers)
                    message += f"; zone markers: {count}"
                if hasattr(response, 'chennal_path_array'):
                    count = len(response.chennal_path_array.markers)
                    message += f"; channel markers: {count}"
                self.result_signal.emit(
                    self.service_name,
                    response.success,
                    message
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
            ensure_rclpy_initialized()
            node = Node(f'mower_qt_param_client_{uuid4().hex}')
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
        elif param['type'] == 'string':
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = str(param['value'])
        else:
            raise ValueError(f"Unsupported parameter type: {param['type']}")

        return ParameterMsg(name=param['name'], value=value)


class MowerQtWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 初始化ROS2
        ensure_rclpy_initialized()
        self.node = Node('mower_qt_node')
        self.parameter_threads = []
        self.service_threads = []
        
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
            ("記錄區域 起始點", "/record_zone_start", Trigger),
            ("記錄區域 結束點", "/record_zone_end", Trigger),
            ("記錄Risk Zone 起始點", "/risk_zone_start", Trigger),
            ("記錄Risk Zone 結束點", "/risk_zone_end", Trigger),
            ("記錄區域 Save", "/save_zone_list", Trigger),
            ("記錄區域 Load", "/load_zone_list", Trigger),
            ("Path Record Info Test", "/get_record_zone_info", Trigger),
            ("取得Record Zone List", "/get_record_zone_list", GetZoneList),
            ("記錄Chennal Start", "/chennal_record_start", Trigger),
            ("記錄Chennal End", "/chennal_record_end", Trigger),
            ("讀取Chennal Path List",
             "/get_chennal_path_list",
             ChennalPathList),
        ]
        
        for btn_text, service_name, service_type in path_record_services:
            btn = self.create_service_button(
                btn_text,
                service_name,
                service_type,
            )
            path_record_layout.addWidget(btn)
        
        layout.addWidget(path_record_group)
        
        # Map Manage 组
        map_manage_group = self.create_group_box("Map Manage")
        map_manage_layout = QVBoxLayout()
        map_manage_group.setLayout(map_manage_layout)
        
        map_manage_services = [
            ("生成多個Zone的Freespace", "/create_free_space", Trigger),
            ("生成多個Zone的Riskspace", "/create_risk_map", Trigger),
            ("建立Chennal Map", "/create_chennal_map", Trigger),
        ]
        
        for btn_text, service_name, service_type in map_manage_services:
            btn = self.create_service_button(
                btn_text,
                service_name,
                service_type,
            )
            map_manage_layout.addWidget(btn)
        
        layout.addWidget(map_manage_group)
        
        # Boustrophedon Coverage 组
        coverage_group = self.create_group_box("Boustrophedon Coverage")
        coverage_layout = QVBoxLayout()
        coverage_group.setLayout(coverage_layout)

        self.create_coverage_parameter_controls(coverage_layout)
        
        coverage_services = [
            ("生成Coverage Path", "/generate_coverage_path", Trigger),
            ("檢查Nav2狀態", "/check_nav_status", Trigger),
            ("取消Nav2", "/cencel_nav2", Trigger),
        ]
        
        for btn_text, service_name, service_type in coverage_services:
            btn = self.create_service_button(
                btn_text,
                service_name,
                service_type,
            )
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

        # Docking 组
        docking_group = self.create_group_box("Docking")
        docking_layout = QVBoxLayout()
        docking_group.setLayout(docking_layout)

        self.create_docking_status_panel(docking_layout)

        docking_services = [
            ("Dock Home", "/mower_dock_home", Trigger),
            ("Undock", "/mower_undock", Trigger),
            ("Cancel Docking", "/mower_cancel_docking", Trigger),
            ("Record Home Dock Pose", "/record_home_dock_pose", Trigger),
            ("刷新Docking狀態", DOCKING_STATUS_SERVICE, Trigger),
        ]

        for btn_text, service_name, service_type in docking_services:
            btn = self.create_service_button(
                btn_text,
                service_name,
                service_type,
            )
            docking_layout.addWidget(btn)

        layout.addWidget(docking_group)

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

        self.coverage_pattern_combo = QComboBox()
        self.coverage_pattern_combo.addItems(['zigzag', 'spiral'])
        self.coverage_pattern_combo.setCurrentText(
            COVERAGE_PARAMETER_DEFAULTS['coverage_pattern'])
        self.coverage_pattern_combo.setFont(QFont('Arial', 10))
        param_layout.addRow("覆蓋模式:", self.coverage_pattern_combo)

        self.zigzag_angle_spinbox = self.create_double_spinbox(
            COVERAGE_PARAMETER_DEFAULTS['zigzag_angle_deg'],
            minimum=0.0,
            maximum=180.0,
            step=5.0,
        )
        param_layout.addRow("Zigzag 角度 angle_deg (°):",
                            self.zigzag_angle_spinbox)

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

    def create_docking_status_panel(self, layout):
        """创建 docking 状态显示."""
        status_widget = QWidget()
        status_layout = QFormLayout()
        status_widget.setLayout(status_layout)

        self.docking_status_labels = {}
        status_fields = [
            ('state', '狀態:'),
            ('dock_id', 'Dock ID:'),
            ('dock_type', 'Dock Type:'),
            ('num_retries', 'Retry:'),
            ('error', '錯誤:'),
            ('charging_confirmed', 'Charging:'),
            ('message', '訊息:'),
        ]

        for key, title in status_fields:
            label = QLabel('-')
            label.setFont(QFont('Arial', 10))
            label.setWordWrap(True)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.docking_status_labels[key] = label
            status_layout.addRow(title, label)

        layout.addWidget(status_widget)

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
    
    def create_service_button(self, text, service_name, service_type=Trigger):
        """创建服务调用按钮"""
        btn = QPushButton(text)
        btn.setMinimumHeight(40)
        btn.setFont(QFont('Arial', 10))
        btn.clicked.connect(
            lambda: self.call_service(service_name, text, service_type)
        )
        return btn
    
    def call_service(self, service_name, button_text, service_type=Trigger):
        """调用ROS2服务"""
        if service_name == "/generate_coverage_path":
            self.generate_coverage_path_with_current_parameters(button_text)
            return

        self.start_service_call(service_name, button_text, service_type)

    def start_service_call(self, service_name, button_text,
                           service_type=Trigger, request_data=None):
        """启动ROS2服务调用线程"""
        self.log_message(f"正在调用服务: {service_name} ({button_text})", "INFO")
        
        # 创建并启动服务调用线程
        thread = ServiceCallThread(service_name, service_type, request_data)
        thread.result_signal.connect(self.on_service_result)
        thread.finished.connect(
            lambda thread=thread: self.cleanup_service_thread(thread)
        )
        self.service_threads.append(thread)
        thread.start()

    def generate_coverage_path_with_current_parameters(self, button_text):
        """先套用当前覆盖参数，再生成覆盖路径."""
        pattern = self.coverage_pattern_combo.currentText()
        self.log_message(
            f"生成 Coverage Path 前先套用目前覆蓋模式: {pattern}",
            "INFO"
        )

        def on_complete(success, messages):
            if success:
                self.log_message(
                    "覆盖参数已套用，开始生成 Coverage Path",
                    "INFO"
                )
                self.start_service_call("/generate_coverage_path", button_text)
                return

            failed = [
                f"{service_name}: {message}"
                for service_name, ok, message in messages
                if not ok
            ]
            self.log_message(
                "覆盖参数套用失败，已取消生成 Coverage Path: "
                + "; ".join(failed),
                "ERROR"
            )

        self.apply_coverage_parameters(on_complete=on_complete)

    def apply_coverage_parameters(self, on_complete=None):
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
                'name': 'zigzag_angle_deg',
                'type': 'double',
                'value': self.zigzag_angle_spinbox.value(),
            },
            {
                'name': 'unknown_as_obstacle',
                'type': 'bool',
                'value': self.unknown_as_obstacle_checkbox.isChecked(),
            },
            {
                'name': 'coverage_pattern',
                'type': 'string',
                'value': self.coverage_pattern_combo.currentText(),
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
        pending = None
        if on_complete is not None:
            pending = {
                'remaining': len(targets),
                'success': True,
                'messages': [],
            }

            def handle_parameter_result(service_name, success, message):
                pending['remaining'] -= 1
                pending['success'] = pending['success'] and success
                pending['messages'].append((service_name, success, message))
                if pending['remaining'] == 0:
                    on_complete(pending['success'], pending['messages'])

        for service_name, params in targets:
            thread = ParameterSetThread(service_name, params)
            thread.result_signal.connect(self.on_parameter_result)
            if on_complete is not None:
                thread.result_signal.connect(handle_parameter_result)
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
        self.zigzag_angle_spinbox.setValue(
            COVERAGE_PARAMETER_DEFAULTS['zigzag_angle_deg'])
        self.inflate_radius_spinbox.setValue(
            COVERAGE_PARAMETER_DEFAULTS['inflate_radius_m'])
        self.unknown_as_obstacle_checkbox.setChecked(
            COVERAGE_PARAMETER_DEFAULTS['unknown_as_obstacle'])
        self.coverage_pattern_combo.setCurrentText(
            COVERAGE_PARAMETER_DEFAULTS['coverage_pattern'])
        self.log_message("覆盖参数输入值已还原为当前程式预设", "INFO")

    def cleanup_parameter_thread(self, thread):
        """清理已完成的参数设置线程"""
        if thread in self.parameter_threads:
            self.parameter_threads.remove(thread)

    def cleanup_service_thread(self, thread):
        """清理已完成的服务调用线程"""
        if thread in self.service_threads:
            self.service_threads.remove(thread)

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

        self.start_service_call(
            service_name,
            f"Zone ID: {zone_id}",
            service_type=ZoneExecPath,
            request_data={'zone_id': zone_id},
        )
    
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

        if service_name == DOCKING_STATUS_SERVICE and success:
            self.update_docking_status(message)
        elif service_name in DOCKING_COMMAND_SERVICES and success:
            self.refresh_docking_status()

    def refresh_docking_status(self):
        """刷新 docking 状态摘要"""
        self.start_service_call(
            DOCKING_STATUS_SERVICE,
            "刷新Docking狀態",
            service_type=Trigger,
        )

    def update_docking_status(self, message):
        """把 docking status JSON 显示到面板."""
        if not hasattr(self, 'docking_status_labels'):
            return

        try:
            status = json.loads(message)
        except json.JSONDecodeError:
            self.docking_status_labels['message'].setText(message)
            return

        state = status.get('state', '-')
        dock_id = status.get('dock_id', '-')
        dock_type = status.get('dock_type', '-')
        num_retries = status.get('num_retries', '-')
        error_code = status.get('error_code', 0)
        error_msg = status.get('error_msg', '')
        charging_confirmed = bool(status.get('charging_confirmed', False))

        error_text = str(error_code)
        if error_msg:
            error_text += f" / {error_msg}"

        charging_text = "confirmed" if charging_confirmed else "not confirmed"

        self.docking_status_labels['state'].setText(str(state))
        self.docking_status_labels['dock_id'].setText(str(dock_id))
        self.docking_status_labels['dock_type'].setText(str(dock_type))
        self.docking_status_labels['num_retries'].setText(str(num_retries))
        self.docking_status_labels['error'].setText(error_text)
        self.docking_status_labels['charging_confirmed'].setText(charging_text)
        self.docking_status_labels['message'].setText(
            str(status.get('message', '-'))
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
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    window = MowerQtWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
