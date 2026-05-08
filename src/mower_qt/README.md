# Mower Qt - ROS2 服务控制面板

这是一个使用PyQt5开发的ROS2服务控制GUI应用程序。

## 功能特点

- **图形化界面**：使用PyQt5创建的现代化GUI界面
- **服务调用**：点击按钮即可调用ROS2服务
- **实时日志**：显示服务调用的状态和结果
- **分组管理**：按功能模块分组显示按钮
- **异步调用**：使用多线程避免界面卡顿

## 支持的服务

### Path Record
- 記錄區域 起始點/結束點
- 記錄Risk Zone 起始點/結束點
- 記錄區域 Save/Load
- 記錄Chennal Start/End
- 讀取Chennal Path List

### Map Manage
- 生成多個Zone的Freespace
- 生成多個Zone的Riskspace
- 建立Chennal Map
- 取得Record Zone List

### Boustrophedon Coverage
- 生成Coverage Path
- Waypoint Pub
- 取消Nav2

## 安装依赖

```bash
# 安装PyQt5
pip3 install PyQt5

# 或者通过apt安装
sudo apt-get install python3-pyqt5
```

## 编译

```bash
cd /home/fxrbindi/Desktop/car_ws
colcon build --packages-select mower_qt
source install/setup.bash
```

## 运行

```bash
# 方法1：直接运行
ros2 run mower_qt mower_qt

# 方法2：作为Python模块运行
python3 -m mower_qt.mower_qt
```

## 使用说明

1. 启动应用程序后，界面分为左右两部分
2. 左侧是按钮控制面板，按功能分组
3. 右侧是日志输出区域
4. 点击任意按钮即可调用对应的ROS2服务
5. 日志会显示服务调用的状态：
   - **蓝色**：信息日志
   - **绿色**：成功
   - **红色**：错误
6. 可以点击"清除日志"按钮清空日志显示

## 注意事项

- 确保相关的ROS2服务节点已经启动
- 如果服务不可用，会在日志中显示错误信息
- 服务调用超时时间设置为5秒

## 故障排除

### 服务不可用
- 检查相关节点是否已启动
- 使用 `ros2 service list` 查看可用服务

### PyQt5导入错误
```bash
pip3 install PyQt5 --upgrade
```

### 权限问题
```bash
chmod +x src/mower_qt/mower_qt/mower_qt.py
```

