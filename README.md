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
| `RaycastLidar` | `/auv0/RaycastLidar` | `sensor_msgs/PointCloud2`（x, y, z, intensity, ring） | 点群（`frame_id: map`） |

waterfall 画像の行数は、Scenario のセンサ設定に `"waterfall_length"` を記述して変更してください。（既定値 500）

---

## ライセンス

依存先の [holoocean-ros](https://github.com/byu-holoocean/holoocean-ros) は MIT License（Copyright (c) 2024 BYU-HoloOcean）です。上のパッチは holoocean-ros のコードに追加するものなので、holoocean-ros のライセンスに従います。
