# Boustrophedon Coverage Path Planning

這是一個基於 ROS2 的覆蓋路徑規劃系統，專為自主割草機設計，使用牛耕式（boustrophedon）模式進行路徑規劃。

## 📁 檔案結構

```
boustrophedon_coverage/
├── boustrophedon_coverage/          # 主要程式碼目錄
│   ├── boustrophedon_coverage.py    # 主節點：路徑規劃與執行
│   ├── path_generators/             # 路徑生成器模組
│   │   ├── boustrophedon.py         # 牛耕式路徑生成器
│   │   ├── speiral.py               # 螺旋式路徑生成器
│   │   └── zigzag.py                # Zigzag 路徑生成器
│   └── utils/                       # 工具模組
│       ├── nav_action_client.py     # 導航動作客戶端
│       ├── path_utils.py            # 路徑工具函數（座標轉換、四元數計算）
│       └── zone_map_client.py       # 區域地圖客戶端
├── launch/                          # 啟動文件
│   ├── nav2.launch.py               # Nav2 導航啟動文件
│   └── open_map.launch.py           # 地圖啟動文件
├── config/                          # 配置文件目錄
├── map/                             # 地圖文件
│   ├── map.pgm                      # 地圖圖像
│   └── map.yaml                     # 地圖元數據
└── test/                            # 測試文件
    ├── test_flake8.py               # Flake8 代碼檢查
    ├── test_pep257.py               # Docstring 檢查
    └── test_copyright.py            # 版權檢查
```

## 🔌 Topics

### 訂閱的 Topics (Subscribed)

| Topic 名稱 | 訊息類型 | 描述 |
|-----------|---------|------|
| `/risk_map` | `nav_msgs/OccupancyGrid` | 原始風險地圖 |
| `/risk_map_inflated` | `nav_msgs/OccupancyGrid` | 膨脹後的風險地圖（用於路徑規劃） |

### 發布的 Topics (Published)

| Topic 名稱 | 訊息類型 | 描述 |
|-----------|---------|------|
| `/coverage_path` | `nav_msgs/Path` | 生成的覆蓋路徑 |
| `/coverage_path_markers` | `visualization_msgs/MarkerArray` | 路徑可視化標記（用於 RViz） |
| `/free_space_inflated` | `nav_msgs/OccupancyGrid` | 膨脹後的自由空間地圖 |
| `/risk_map_inflated` | `nav_msgs/OccupancyGrid` | 發布膨脹後的風險地圖 |

## 🛠️ Services

### 提供的 Services (Provided)

| Service 名稱 | 服務類型 | 描述 |
|-------------|---------|------|
| `/generate_coverage_path` | `std_srvs/Trigger` | 生成覆蓋路徑 |
| `/zone_exec_path` | `boustrophedon_coverage_interfaces/srv/ZoneExecPath` | 執行指定區域的路徑 |
| `/cencel_nav2` | `std_srvs/Trigger` | 取消當前的 Nav2 導航任務 |
| `/check_nav_status` | `std_srvs/Trigger` | 檢查導航狀態 |

### 調用的 Services (Called)

| Service 名稱 | 服務類型 | 描述 |
|-------------|---------|------|
| `/record_path_status` | `std_srvs/SetBool` | 設置路徑記錄狀態 |
| `/get_zone_map_list_srv` | `boustrophedon_coverage_interfaces/srv/ZoneMapList` | 獲取區域地圖列表 |

## 🎯 Actions

### Action Clients

| Action 名稱 | Action 類型 | 描述 |
|------------|-----------|------|
| `nav_action` | `nav2_action_interfaces/action/Waypoint` | 導航到路徑點 |
| `nav_action_follow_path` | `nav2_action_interfaces/action/Waypoint` | 跟隨分段路徑 |

## 📦 主要功能模組

### 1. CoveragePlanner (主節點)
- **文件**: `boustrophedon_coverage.py`
- **功能**:
  - 訂閱風險地圖
  - 生成覆蓋路徑
  - 管理路徑執行
  - 與 Nav2 整合

### 2. 路徑生成器 (Path Generators)

#### Boustrophedon (牛耕式)
- **文件**: `path_generators/boustrophedon.py`
- **特點**:
  - 直線來回掃描
  - 支援角度旋轉
  - 可配置條帶寬度和路徑點間距

#### Spiral (螺旋式)
- **文件**: `path_generators/speiral.py`
- **特點**:
  - 從中心向外螺旋
  - 適合不規則區域

#### Zigzag (之字形)
- **文件**: `path_generators/zigzag.py`
- **特點**:
  - 之字形掃描模式
  - 適合矩形區域

### 3. 工具模組 (Utils)

#### NavActionClient
- **文件**: `utils/nav_action_client.py`
- **功能**:
  - 發送導航目標到 action server
  - 處理導航反饋和結果
  - 支援分段路徑執行

#### PathUtils
- **文件**: `utils/path_utils.py`
- **功能**:
  - 歐拉角到四元數轉換
  - 計算兩點間朝向
  - 路徑點格式轉換
  - 地圖兼容性驗證

#### ZoneMapClient
- **文件**: `utils/zone_map_client.py`
- **功能**:
  - 從服務獲取區域地圖
  - 管理區域地圖列表

## 🚀 使用方式

### 1. 生成覆蓋路徑
```bash
ros2 service call /generate_coverage_path std_srvs/srv/Trigger
```

### 2. 執行指定區域的路徑
```bash
ros2 service call /zone_exec_path boustrophedon_coverage_interfaces/srv/ZoneExecPath "zone_id: 'zone_1'"
```

### 3. 取消導航
```bash
ros2 service call /cencel_nav2 std_srvs/srv/Trigger
```

### 4. 檢查導航狀態
```bash
ros2 service call /check_nav_status std_srvs/srv/Trigger
```

## ⚙️ 參數配置

| 參數名稱 | 類型 | 默認值 | 描述 |
|---------|------|-------|------|
| `strip_width_m` | float | 0.2 | 割草機有效割幅（米）|
| `waypoint_spacing_m` | float | 0.1 | 路徑點間距（米）|
| `unknown_as_obstacle` | bool | True | 未知區域是否視為障礙 |
| `use_sim_time` | bool | True | 是否使用模擬時間 |

## 📊 資料流程

```
1. 訂閱 risk_map_inflated
   ↓
2. 調用 /get_zone_map_list_srv 獲取區域地圖
   ↓
3. 為每個區域生成覆蓋路徑
   ↓
4. 發布路徑到 /coverage_path 和 /coverage_path_markers
   ↓
5. 通過 /zone_exec_path 執行路徑
   ↓
6. 發送目標到 nav_action_follow_path
   ↓
7. Nav2 執行路徑導航
```

## 🔧 依賴項

- ROS2 (Humble/Foxy)
- Nav2
- geometry_msgs
- nav_msgs
- std_msgs
- std_srvs
- visualization_msgs
- boustrophedon_coverage_interfaces (自定義接口)
- nav2_action_interfaces
- nav2_simple_commander

## 📝 自定義訊息/服務

需要 `boustrophedon_coverage_interfaces` 套件，包含：
- `ZoneExecPath.srv` - 區域執行路徑服務
- `ZoneMapList.srv` - 區域地圖列表服務
- `ZoneMap.msg` - 區域地圖訊息

## 🎨 可視化

在 RViz 中可以查看：
- 覆蓋路徑線條（不同區域用不同顏色）
- 路徑方向箭頭
- 風險地圖
- 膨脹後的地圖

## 📄 代碼風格

所有 Python 代碼遵循 PEP 8 規範：
- 使用單引號
- 行長度限制 79 字符
- 完整的 docstring 文檔
- 類型提示

