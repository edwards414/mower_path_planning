# Mower Qt Service Console Frontend Function Spec

## 1. 文件目的

本文件提供前端開發使用，描述目前 `mower_qt` 套件中的 `mower_qt` 工具功能，並整理成可轉換為 Web Frontend 或新版 GUI 的功能規格。

本文件主要根據以下來源整理：

- `tools.txt`
- `src/mower_qt`

另外為了避免接口定義與實際後端不一致，本文有交叉比對目前專案中的 service 實作。

## 2. 功能範圍

本文件目前只保留 `mower_qt` 對應的前端規格：

1. `mower_qt`
   - ROS2 服務控制台
   - 用途是把一組 ROS2 service 操作做成按鈕介面

`stm32_uart_monitor.py` 相關前端規格已自本文件移除；若未來需要重做 UART 監控頁，建議另立文件維護。

## 3. 相關啟動與依賴

根據 `tools.txt`，前端操作前通常會先啟動以下後端節點或 launch：

| 類別 | 指令 | 說明 |
|---|---|---|
| 地圖/導航 | `ros2 launch nav2_gps_waypoint_follower gps_waypoint_follower.launch.py` | 啟動 Nav2 與地圖導航 |
| 系統測試 | `ros2 launch nav2_gps_waypoint_follower system_test.launch.py` | 啟動整體測試流程 |
| 路徑記錄 | `ros2 run path_record path_record` | 提供區域記錄、risk zone、channel path 等服務 |
| 地圖處理 | `ros2 run maphub map_manage` | 提供 free space、risk map、channel map 等服務 |
| 覆蓋路徑 | `ros2 run boustrophedon_coverage boustrophedon_coverage` | 提供 coverage path 生成與執行服務 |
| 控制面板 | `ros2 run mower_qt mower_qt` | 啟動 ROS2 服務控制台 |

前端要注意：

- 若後端節點未啟動，service 呼叫會失敗。
- `mower_qt` 目前的行為是先等待 service 3 秒，送出後最多等 5 秒回應。

## 4. 前端資訊架構

建議畫面結構：

- 左側或主區域：功能群組按鈕
- 右側：操作日誌
- 上方可加一個系統狀態列，顯示 ROS 連線狀態、最近一次操作結果、目前區域 ID

建議群組：

1. Path Record
2. Map Manage
3. Boustrophedon Coverage
4. System Launch / Tool Entry

## 5. 作業控制台功能規格

### 5.1 共通互動規則

所有 service 型操作都應遵守以下規則：

- 操作是非同步的，不能卡住整個 UI。
- 點擊後要進入 loading 狀態。
- 成功與失敗都必須寫入操作日誌。
- 建議每顆按鈕在 request 完成前暫時 disabled，避免重複送出。
- 共通日誌格式建議：
  - `[timestamp] [INFO] 開始呼叫 /service_name`
  - `[timestamp] [SUCCESS] /service_name: message`
  - `[timestamp] [ERROR] /service_name: message`

### 5.2 Path Record 模組

用途：記錄 mowing zone、risk zone、channel path，並支援保存/載入。

| UI 功能 | 後端接口 | 類型 | Request | Response | 說明 |
|---|---|---|---|---|---|
| 記錄區域 起始點 | `/record_zone_start` | `std_srvs/Trigger` | 無 | `success`, `message` | 開始記錄一般區域 |
| 記錄區域 結束點 | `/record_zone_end` | `std_srvs/Trigger` | 無 | `success`, `message` | 結束記錄一般區域 |
| 記錄 Risk Zone 起始點 | `/risk_zone_start` | `std_srvs/Trigger` | 無 | `success`, `message` | 開始記錄 risk zone |
| 記錄 Risk Zone 結束點 | `/risk_zone_end` | `std_srvs/Trigger` | 無 | `success`, `message` | 結束記錄 risk zone |
| 記錄區域 Save | `/save_zone_list` | `std_srvs/Trigger` | 無 | `success`, `message` | 保存區域資料 |
| 記錄區域 Load | `/load_zone_list` | `std_srvs/Trigger` | 無 | `success`, `message` | 載入區域資料 |
| 記錄區域 Risk Save | `/risk_zone_save` | `std_srvs/Trigger` | 無 | `success`, `message` | 保存 risk zone |
| Path Record Info | `/get_record_zone_info` | `std_srvs/Trigger` | 無 | `success`, `message` | 取得區域記錄資訊 |
| 記錄 Chennal Start | `/chennal_record_start` | `std_srvs/Trigger` | 無 | `success`, `message` | 開始記錄 channel path |
| 記錄 Chennal End | `/chennal_record_end` | `std_srvs/Trigger` | 無 | `success`, `message` | 結束記錄 channel path |
| 讀取 Chennal Path List | `/get_chennal_path_list` | `path_record_interface/srv/ChennalPathList` | 無 | `success`, `message`, `chennal_path_array` | 目前桌面版 UI 把它當 `Trigger`，這是錯的，前端不要照抄 |
| 取得 Record Zone List | `/get_record_zone_list` | `boustrophedon_coverage_interfaces/srv/GetZoneList` | 無 | `success`, `message`, `zone_list` | 目前桌面版 UI 寫成 `/get_record_zone_list_srv`，這是錯的 |

前端建議：

- 對 `GetZoneList` 與 `ChennalPathList`，不要只顯示 success/message，應保留原始 payload，供地圖或列表 UI 使用。
- 若未來要接地圖，可直接把 `MarkerArray` 轉成前端圖層資料。

### 5.3 Map Manage 模組

用途：依據已記錄的 zone / risk zone / channel path 產生對應地圖資料。

| UI 功能 | 後端接口 | 類型 | Request | Response | 說明 |
|---|---|---|---|---|---|
| 生成多個 Zone 的 Freespace | `/create_free_space` | `std_srvs/Trigger` | 無 | `success`, `message` | 生成可通行空間 |
| 生成多個 Zone 的 Riskspace | `/create_risk_map` | `std_srvs/Trigger` | 無 | `success`, `message` | 生成風險區域地圖 |
| 建立 Chennal Map | `/create_chennal_map` | `std_srvs/Trigger` | 無 | `success`, `message` | 由 channel path 建立 channel map |

前端操作提示建議：

- `create_free_space` 應視為前置步驟。
- `create_risk_map`、`create_chennal_map` 可在 UI 上提示其依賴 `free_space` 已生成。
- 若後端已提供狀態 topic，未來可在前端加入進度條，但目前桌面版沒有做。

### 5.4 Boustrophedon Coverage 模組

用途：生成覆蓋路徑、執行指定區域路徑、取消目前導航。

| UI 功能 | 後端接口 | 類型 | Request | Response | 說明 |
|---|---|---|---|---|---|
| 生成 Coverage Path | `/generate_coverage_path` | `std_srvs/Trigger` | 無 | `success`, `message` | 生成覆蓋路徑 |
| 執行 Zone Path | `/zone_exec_path` | `boustrophedon_coverage_interfaces/srv/ZoneExecPath` | `zone_id: int32` | `success`, `message` | 執行指定區域路徑 |
| 取消 Nav2 | `/cencel_nav2` | `std_srvs/Trigger` | 無 | `success`, `message` | 取消目前導航任務 |
| 檢查導航狀態 | `/check_nav_status` | `std_srvs/Trigger` | 無 | `success`, `message` | 後端有提供，但目前桌面版 UI 尚未做 |

`ZoneExecPath` 請求格式：

```json
{
  "zone_id": 1
}
```

前端欄位規則：

- `zone_id` 為整數
- 目前桌面版允許 `0 ~ 999`
- 建議前端沿用整數輸入框，不要用自由文字輸入

### 5.5 System Launch / Tool Entry

`tools.txt` 內還列出多個常用啟動指令，前端如果要做系統入口頁，可作為快捷操作或操作說明：

| 名稱 | 指令 |
|---|---|
| 地圖 nav2 | `ros2 launch nav2_gps_waypoint_follower gps_waypoint_follower.launch.py` |
| test system | `ros2 launch nav2_gps_waypoint_follower system_test.launch.py` |
| system test | `ros2 launch mower_bringup system_test.launch.py launch_sim:=true use_sim_time:=true` |
| rviz | `ros2 launch nav2_gps_waypoint_follower rviz.launch.py` |
| 控制面板 | `ros2 run mower_qt mower_qt` |

若前端不是跑在本機，這些啟動指令不應直接做成前端按鈕，而應由後端代理執行。

## 6. 前端元件拆分建議

建議元件：

- `ServiceGroupCard`
- `ServiceActionButton`
- `ZoneExecForm`
- `OperationLogPanel`
- `SystemCommandList`

## 7. 已知差異與待確認事項

這一段很重要，前端不要直接照現在桌面版 UI 的所有按鈕名稱與接口硬做。

| 項目 | 現況 | 建議 |
|---|---|---|
| `/get_record_zone_list_srv` | 桌面版 UI 使用這個名稱，但後端實際存在的是 `/get_record_zone_list` | 前端以 `/get_record_zone_list` 為準 |
| `/get_chennal_path_list` 類型 | `tools.txt` 和桌面版 UI 都把它當 `Trigger` | 實際是 `path_record_interface/srv/ChennalPathList` |
| `Waypoint Pub` | 桌面版 UI 把它當 service `/waypoint_pub` | 目前 source 只看到 `waypoint_pub` 可執行節點，沒看到對應 service，前端先不要當 service button 做 |
| `ZoneExecPath` 範例 | 某些文件示例曾用字串型 `zone_1` | 目前 `.srv` 定義是 `int32 zone_id`，前端請用整數 |
| 拼字 | 目前後端存在 `chennal`、`cencel` 等命名 | 前端顯示文案可修正，但 API 名稱必須保留原拼字 |

## 8. 最小可交付版本建議

若前端要先做 MVP，建議優先順序如下：

1. 作業控制台
   - Path Record 基本 Trigger 按鈕
   - Map Manage 基本 Trigger 按鈕
   - Coverage Path 生成、`zone_exec_path`、`cencel_nav2`
   - 操作日誌
2. 第二階段
   - `GetZoneList`、`ChennalPathList` 的資料可視化
   - 地圖整合
   - 系統啟動頁或後端代理控制
