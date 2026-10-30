# holoocean_viz

[HoloOcean](https://github.com/byu-holoocean/HoloOcean) のセンサ出力を、RViz2 や rqt で見やすい ROS 2 メッセージに変換する。

- `IMUSensor` のトピックに TF を付与して再配信し、RViz2 で表示できるようにする。
- `SidescanSonar` の値を ROS 2 トピックとして配信する。
-  `RaycastLidar` の値を ROS 2 トピックとして配信する。

> このパッケージは単体では動きません。
> [byu-holoocean/holoocean-ros](https://github.com/byu-holoocean/holoocean-ros) の `holoocean_main`（シミュレータノード）と `holoocean_interfaces` に依存しており、それらに従属する拡張パッケージです。
> トピック名、シナリオ JSON の書式、Encoder の基底クラス `SensorPublisher` は、すべて holoocean-ros の仕様に合わせています。

---

## 想定実行環境

| 項目 | バージョン / 内容 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| Python | 3.10 |
| HoloOcean | 2.3.0（Ocean パッケージ） |
| holoocean-ros | `main` ブランチ（コミット `f0c78af` 時点）+ 後述のパッチ |
| RViz プラグイン | `rviz_imu_plugin`（`ros-humble-imu-tools`） |

---

## 機能

### 1. `imu_viz` ノード

HoloOcean の `IMUSensor` は `frame_id: COM` で配信されますが、このフレームには TF がないため、RViz2 では表示できません。`imu_viz` は次の処理だけを行います。

- static TF `map → auv0/imu` を配信する
- 受け取った `sensor_msgs/Imu` を、`frame_id` だけ `auv0/imu` に置き換えて再配信する（値は変更しない）

| 種別 | トピック / フレーム | 型 |
|---|---|---|
| 入力 | `/auv0/IMUSensor` | `sensor_msgs/Imu` |
| 出力 | `/auv0/IMUSensor/rviz` | `sensor_msgs/Imu` |
| TF | `map → auv0/imu`（static） | – |

パラメータ（[config/imu_viz.yaml](config/imu_viz.yaml)）：

| 名前 | 既定値 | 説明 |
|---|---|---|
| `imu_topic` | `/auv0/IMUSensor` | 入力トピック |
| `output_topic` | `<imu_topic>/rviz` | 出力トピック |
| `world_frame` | `map` | TF の親フレーム |
| `imu_frame` | `auv0/imu` | TF の子フレーム |




### 2. 追加の Encoder（[holoocean_viz/encoders.py](holoocean_viz/encoders.py)）

`holoocean_main` が起動時に読み込み、シナリオ JSON で `"ros_publish": true` にしたセンサを、標準のセンサと同じように配信します。トピック名は `/<agent_name>/<sensor_name><suffix>` です。

| センサ | トピック（例） | 型 | 内容 |
|---|---|---|---|
| `SidescanSonar` | `/auv0/SidescanSonar` | `sensor_msgs/Image`（32FC1, 1×RangeBins） | 1 回分の生の強度値 |
| | `/auv0/SidescanSonarWaterfall` | `sensor_msgs/Image`（mono8） | 直近 N 回分出力を積み上げた waterfall |
| `RaycastLidar` | `/auv0/RaycastLidar` | `sensor_msgs/PointCloud2`（x, y, z, intensity, ring） | センサ座標の点群（`frame_id: <sensor_name>`）。キャプチャのあった tick だけ配信 |

waterfall 画像の行数は、Scenario のセンサ設定に `"waterfall_length"` を記述して変更してください。（既定値 500）

### 3. `lidar_bridge` ノード（LiDAR-inertial SLAM 用）

`RaycastLidar` の点群はセンサ座標ですが、点ごとの時刻がなく、TF もないため、そのままでは SLAM に使えません（HoloOcean のドキュメントにはワールド座標とありますが、2.3.0 で確認したところ実際はセンサ座標でした）。`lidar_bridge` は、実機の Velodyne 型 LiDAR + IMU と同じ形式に変換し、[FAST-LIO2](https://github.com/hku-mars/FAST_LIO) / [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) にそのまま渡せるようにします。

- 点群を Velodyne 形式に詰め直す（ワールド座標で来る場合は `input_frame: world` で、同じ tick の真値の位置姿勢を使ってセンサ座標に変換する）
- 1 回転分（`scan_period`）のキャプチャをまとめ、`x, y, z, intensity, ring(uint16), time(float32, スキャン開始からの秒)` の点群として配信する
- IMU を `auv0/imu_link` で再配信する。LIO-SAM は姿勢付き IMU が必要なので、`orientation` を真値で埋める（`imu_orientation: ground_truth`）
- TF と、比較用の真値の軌跡を配信する

| 種別 | トピック / フレーム | 型 |
|---|---|---|
| 入力 | `/auv0/RaycastLidar`, `/auv0/DynamicsSensorOdom`, `/auv0/IMUSensor` | `PointCloud2`, `Odometry`, `Imu` |
| 出力 | `/auv0/lidar/points`（`auv0/lidar_link`） | `sensor_msgs/PointCloud2` |
| | `/auv0/imu/data`（`auv0/imu_link`） | `sensor_msgs/Imu` |
| | `/auv0/ground_truth/odom`, `/auv0/ground_truth/path` | `Odometry`, `Path` |
| TF | `auv0/base_link → auv0/lidar_link`, `auv0/base_link → auv0/imu_link`（static） | – |
| | `map → auv0/base_link`（真値。LIO-SAM 使用時は `auv0/base_link_gt`） | – |
| | `map → camera_init` / `map → lio_odom`（static、初期位置。SLAM の結果を真値と重ねて表示するため） | – |

パラメータは [config/lidar_bridge.yaml](config/lidar_bridge.yaml) です。`lidar_xyz` / `lidar_rpy_deg` / `imu_xyz` / `imu_rpy_deg` は、シナリオ JSON に書いたセンサの `location` / `rotation` と同じ値にしてください。

#### 実行

```bash
# シミュレータ + lidar_bridge + FAST-LIO2 + RViz2
ros2 launch holoocean_viz lidar_slam_launch.py with_sim:=true slam:=fast_lio

# LIO-SAM の場合
ros2 launch holoocean_viz lidar_slam_launch.py with_sim:=true slam:=lio_sam

# SLAM なし（変換結果だけ確認）
ros2 launch holoocean_viz lidar_slam_launch.py with_sim:=true slam:=none
```

- `with_sim:=true` では、軽量なシナリオ [config/scenarios/blueROV_lio.json](config/scenarios/blueROV_lio.json) を使います（`ticks_per_sec: 100`、ソナー・カメラなし）。別のシナリオは `scenario:=<JSON の絶対パス>` で指定します。
- SLAM のパラメータは [config/slam/fast_lio_holoocean.yaml](config/slam/fast_lio_holoocean.yaml)、[config/slam/lio_sam_holoocean.yaml](config/slam/lio_sam_holoocean.yaml) です。LiDAR の設定（`Channels`, `RotationFrequency`, `PointsPerSecond`）を変えたら、`scan_line` / `N_SCAN` / `Horizon_SCAN` なども合わせてください。
- FAST-LIO2 と LIO-SAM は別途ソースからビルドが必要です（FAST-LIO2 の ROS 2 版は `livox_ros_driver2`、LIO-SAM は GTSAM に依存）。

#### 注意

- IMU のレートは `ticks_per_sec` と同じです。LIO 系は 100 Hz 以上が望ましいため、30 Hz のシナリオでは精度が落ちます。
- LIO-SAM 用の IMU 姿勢は真値で埋めています（シミュレーション上の簡略化）。
- 実際の LiDAR は水中ではほとんど使えないため、水中の機体での結果はシミュレーション上の検証として扱ってください。

---

## ライセンス

依存先の [holoocean-ros](https://github.com/byu-holoocean/holoocean-ros) は MIT License（Copyright (c) 2024 BYU-HoloOcean）です。上のパッチは holoocean-ros のコードに追加するものなので、holoocean-ros のライセンスに従います。
