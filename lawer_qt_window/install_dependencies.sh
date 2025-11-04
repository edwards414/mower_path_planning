#!/bin/bash

# Lawer Qt Window - 依赖安装脚本
# 这个脚本将安装运行Qt6 ROS2应用程序所需的所有依赖

set -e

echo "=========================================="
echo "  Lawer Qt Window - 依赖安装脚本"
echo "=========================================="

# 检查是否为root用户
if [[ $EUID -eq 0 ]]; then
   echo "请不要以root用户运行此脚本"
   exit 1
fi

# 检查操作系统
if [[ ! -f /etc/os-release ]]; then
    echo "错误: 无法检测操作系统版本"
    exit 1
fi

source /etc/os-release

echo "检测到操作系统: $PRETTY_NAME"

# 检查Ubuntu版本
if [[ "$ID" != "ubuntu" ]]; then
    echo "警告: 此脚本专为Ubuntu设计，其他发行版可能需要手动安装"
fi

# 更新包列表
echo "正在更新包列表..."
sudo apt update

# 安装Qt6开发包
echo "正在安装Qt6开发包..."
sudo apt install -y \
    qt6-base-dev \
    qt6-tools-dev \
    libqt6core5compat6-dev \
    qt6-base-dev-tools \
    libqt6svg6-dev \
    libqt6opengl6-dev

# 检查ROS2安装
echo "检查ROS2安装..."
ROS2_FOUND=false

for ros_version in rolling jazzy humble; do
    if [[ -f "/opt/ros/$ros_version/setup.bash" ]]; then
        echo "找到ROS2 $ros_version"
        ROS2_FOUND=true
        ROS2_VERSION=$ros_version
        break
    fi
done

if [[ "$ROS2_FOUND" == "false" ]]; then
    echo "错误: 未找到ROS2安装"
    echo "请先安装ROS2，然后重新运行此脚本"
    echo "安装指南: https://docs.ros.org/en/rolling/Installation.html"
    exit 1
fi

# 安装ROS2开发工具
echo "正在安装ROS2开发工具..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake

# 检查rosdep初始化
if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
    echo "初始化rosdep..."
    sudo rosdep init
fi

echo "更新rosdep..."
rosdep update

# 创建环境设置脚本
SETUP_SCRIPT="$HOME/.lawer_qt_setup.sh"
echo "创建环境设置脚本: $SETUP_SCRIPT"

cat > "$SETUP_SCRIPT" << EOF
#!/bin/bash
# Lawer Qt Window 环境设置脚本

# 设置ROS2环境
source /opt/ros/$ROS2_VERSION/setup.bash

# 设置工作空间环境 (如果存在)
if [[ -f "\$HOME/car_ws/install/setup.bash" ]]; then
    source "\$HOME/car_ws/install/setup.bash"
elif [[ -f "\$(pwd)/install/setup.bash" ]]; then
    source "\$(pwd)/install/setup.bash"
fi

# 设置Qt6环境变量
export QT_QPA_PLATFORM=xcb
export QT_LOGGING_RULES="*.debug=false"

echo "ROS2 $ROS2_VERSION 环境已设置"
echo "Qt6 环境已设置"
EOF

chmod +x "$SETUP_SCRIPT"

# 添加到bashrc (可选)
if ! grep -q "lawer_qt_setup.sh" "$HOME/.bashrc"; then
    echo ""
    echo "是否要将环境设置添加到 ~/.bashrc? (y/n)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "" >> "$HOME/.bashrc"
        echo "# Lawer Qt Window 环境设置" >> "$HOME/.bashrc"
        echo "source $SETUP_SCRIPT" >> "$HOME/.bashrc"
        echo "已添加到 ~/.bashrc"
    fi
fi

# 验证安装
echo ""
echo "验证安装..."

# 检查Qt6
if pkg-config --exists Qt6Core; then
    QT6_VERSION=$(pkg-config --modversion Qt6Core)
    echo "✓ Qt6 已安装 (版本: $QT6_VERSION)"
else
    echo "✗ Qt6 安装可能有问题"
fi

# 检查CMake
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
    echo "✓ CMake 已安装 (版本: $CMAKE_VERSION)"
else
    echo "✗ CMake 未安装"
fi

# 检查colcon
if command -v colcon &> /dev/null; then
    echo "✓ colcon 已安装"
else
    echo "✗ colcon 未安装"
fi

echo ""
echo "=========================================="
echo "  安装完成!"
echo "=========================================="
echo ""
echo "下一步:"
echo "1. 重新启动终端或运行: source $SETUP_SCRIPT"
echo "2. 进入工作空间目录"
echo "3. 运行: colcon build --packages-select lawer_qt_window"
echo "4. 运行: ros2 run lawer_qt_window lawer_qt_window_node"
echo ""
echo "如果遇到问题，请查看 README.md 文件"
echo ""
