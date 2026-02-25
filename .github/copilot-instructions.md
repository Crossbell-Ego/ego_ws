# ROS2 專案固定規格與規則（僅此 workspace 生效）

## 永久系統規格（每次回答必參考）
- 主控：NVIDIA Jetson Orin Nano Super Developer Kit 8GB
- ROS2：Humble
- Workspace：~/ego_ws
- 系統使用者：jetson
- 地區：台灣（不用翻牆）
- ROBOT_TYPE=X3plus
- RPLIDAR_TYPE=4ROS（YDLIDAR 4ROS，TOF，30m，100KLux）
- CAMERA_TYPE=astraplus（Orbbec Astra Pro Plus）
- 手臂相機：Orbbec Gemini Plus
- 不使用 Docker
- RVIZ 在 VMware 虛擬機上運行
- jetson ip : 192.168.18.15
- VM ip :192.168.18.19

### 這套系統的硬體規格詳細資訊如下：

- 主控單元： 採用 NVIDIA Jetson Orin Nano Super 作為運算核心。
- 雷射雷達 (LiDAR)： 配備 YDLIDAR 4ROS，這是一款高性能 TOF 雷達，掃描半徑達 30 公尺，並具備 100KLux 的抗強光能力。
- 深度相機： 使用奧比中光 (Orbbec) 的 Astra Pro Plus 深度攝影機。
- 機械臂： 搭載一具 6 自由度 (6DOF) 的總線伺服舵機，並包含角度回讀功能。
- 馬達： 驅動系統使用 520 型高功率馬達。
- 感測器： 內建九軸 IMU，整合了加速度計、陀螺儀與磁力計。
- 顯示與操作： 配備一個 7 吋的 HD 觸控螢幕，用於顯示和互動操作。
- 語音模組： 包含一個透過 USB 序列通訊的語音辨識互動模組。
- 輪型與底盤： 底盤結構為全鋁合金，搭配 4 個 65mm 的麥克納姆輪和搖擺懸吊設計。
- 電源： 採用 12V 和 5V 的多通道供電方案。
- 擴展介面： 提供豐富的擴充選項，包括一個 4 埠的 USB 3.0 HUB、microUSB 接口以及 CAN Bus。
- 控制板： 控制板型號為 YB-ERF01-V2.0。
- 手臂上相機： 在機械臂上另外搭載了奧比中光 (Orbbec) 的 Gemini Plus 深度攝影機。

## FastDDS / ROS2 網路設定

- Jetson 與 VMware 之間使用 FastDDS unicast 點對點通訊（VMware 不支援 multicast）
- 兩台機器的 FastRTPS profile 路徑統一為 `~/fastrtps_profile.xml`
- 環境變數：`FASTRTPS_DEFAULT_PROFILES_FILE=~/fastrtps_profile.xml`（已寫入兩台 ~/.bashrc）

| 機器 | 設定檔路徑 | 綁定 IP |
|------|-----------|---------|
| Jetson | ~/fastrtps_profile.xml | 192.168.18.15 |
| VMware | ~/fastrtps_profile.xml | 192.168.18.19 |

- RVIZ 設定檔：`src/egocar_nav/rviz/map.rviz`

## 回覆/指令規則（Simple is best）
- 任何要我輸入指令時，必須明確寫「本地終端機」或「DOCKER終端機」（本專案預設不使用 Docker）。
- 代碼與步驟盡量簡單，提供可一鍵複製、可直接跑的版本。
- 安裝 ROS2 依賴：優先 sudo apt install ...
- 安裝 Python 套件：禁止 sudo pip；必要時用 pip install <pkg> --user
- 一律用繁體中文回覆。
