
#  RZ/A2M用の Micro-XRCE-DDS-Client を用いたROS 2環境のデモ

# デモの概要

RZ/A2M用の Micro-XRCE-DDS-Client のデモとして、シマフジの RZ/A2M 搭載ボード SEMB-1451/1452 に接続されたロボットアームを MoveIt! によって制御します。

# 1. デモ実行環境
デモの実行に必要な項目は以下の通りです。  

### [ハードウェア]  
(1) SEMB-1451/1452 1組み  
作成したイメージが動作するターゲットボードです。  

SEMB-1451/1452 はシマフジ電機株式会社より購入できます。
  
JPN : http://www.shimafuji.co.jp/products/1767
  
ENG : http://www.shimafuji.co.jp/en/products/1522


(2) Windows PC 1台  
SEMB-1451/1452 へファームウェアを書き込むために使用します。  

(3) Linux PC 1台  
ROS/ROS 2 環境を動作させるために使用します。  
動作確認を行った OS とバージョンは「Ubuntu 18.04 LTS」です。  

(4) J-Link, USB ケーブル 1セット  
ファームウェアを書き込むために使用します。  

(5) Ethernet HUB 1台  
Linux PC - SEMB-1451/1452 間でローカルネットワークを構築するために使用します。  

(6) Ethernet ケーブル 2本  
Linux PC - SEMB-1451/1452 間でローカルネットワークを構築するために使用します。  

(7) OpenMANIPULATOR-X 1台  
ROBOTIS 社製のロボットアームです。以下「Robot Arm」とします。  

本デモでは、上記(1)～(7)の機器を以下のように接続します。  

```
                         +---------------+                 +--------------+
       +-----------------+     J-Link    |                 |  Linux PC *2 |
       |                 +-------+-------+                 +------+-------+
       | USB                     | JTAG                           | Ethernet
       |                         |                                | 192.168.2.101
+------+-------+         +-------+--------+    Ethernet    +------+-------+
|  Windows PC  +---------+ SEMB-1451/1452 +----------------+ Ethernet HUB |
+--------------+  UART   +-------+--------+  192.168.2.52  +--------------+
                (SCIFA4)         |
                   *1            |      UART(SCIFA2)*3     +--------------+
                                 +-------------------------+  Robot Arm   |
                                                           +--------------+
```

*1. USB Serial は以下のコネクタに接続してください。  
    CN17[ボーレート: 115200bps]  
*2. Linux PC はローカルネットワークのみの環境にし、インターネットに接続しないでください。  
    また、Linux PC の IPv6設定は無効にしてください。  
*3. 基板上のCN15のシルクから、GND(pin 4), 12V(pin 3), Signal(pin 2)の順番で接続してください(pin 1は未接続)。


### [ソフトウェア]  

#### Linux PC
以下のソフトウェアがインストールされている必要があります。  

(1) ROS (Melodic)  
パッケージインストールを前提としています。  
インストール方法については Web ページ上のドキュメントを参照してください。  
http://wiki.ros.org/melodic/Installation/Ubuntu

(2) ROS 2 (Crystal Clemmys)  
パッケージインストールを前提としています。  
インストール方法については Web ページ上のドキュメントを参照してください。  
https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/  

(3) Micro-XRCE-DDS-Agent  
インストール方法は「2. Micro-XRCE-DDS-Agent のビルド」で説明します。  
動作確認を行ったバージョンは「1.1.0」です。  

(4) ロボットモデル制御アプリケーション  
アーカイブ「ROS1_OpenManipulatorX_EVK-master.zip」(このリポジトリ)です。「4.1. ソースの展開」で使用します。  
以下の URL から取得してください。本手順では、ホームディレクトリに配置されているものとします。    

https://github.com/JiGoRo1969/ROS1_OpenManipulatorX_EVK/archive/master.zip


#### Windows PC
以下のソフトウェアがインストールされている必要があります。  

(1) e2studio  
インストール方法については Web ページ上のドキュメントを参照してください。  
https://www.renesas.com/jp/ja/products/software-tools/tools/ide/e2studio.html  
動作確認を行ったバージョンは「7.5.0」です。  

(2) FreeRTOS ソース一式  
アーカイブ「ROS2_RZA2M_MoveIt1-master.zip」です。「5.1. プロジェクトのインポート」で使用します。  
以下の URL から取得してください。  

https://github.com/JiGoRo1969/ROS2_RZA2M_MoveIt1/archive/master.zip

# 2. Micro-XRCE-DDS-Agent のビルド  

## 2.1. ソースの取得  
以下のコマンドを実行してください。  
```
$ cd ~
$ git clone -b v1.1.0 https://github.com/JiGoRo1969/Micro-XRCE-DDS-Agent
$ cd Micro-XRCE-DDS-Agent
$ git submodule update --init --recursive
```


## 2.2. spdlog のインストール
Micro-XRCE-DDS-Agent ビルドのために、以下の環境を準備します。  
```
$ cd ~/Micro-XRCE-DDS-Agent
$ pushd thirdparty/spdlog/
$ mkdir build; cd build
$ cmake -DSPDLOG_BUILD_EXAMPLES=OFF -DSPDLOG_BUILD_BENCH=OFF -DSPDLOG_BUILD_TESTS=OFF ..
$ sudo make install
$ popd
```

## 2.3. CLI11 のインストール 
Micro-XRCE-DDS-Agent ビルドのために、以下の環境を準備します。  
```
$ cd ~/Micro-XRCE-DDS-Agent
$ pushd thirdparty/CLI11/
$ mkdir build; cd build
$ cmake -DCLI11_TESTING=OFF -DCLI11_EXAMPLES=OFF ..
$ sudo make install
$ popd
```

## 2.4. 本体のビルドとインストール
以下のコマンドを実行してください。  
```
$ cd ~/Micro-XRCE-DDS-Agent
$ mkdir build; cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig /usr/local/lib
```

# 3. Linux 側環境構築手順

## 3.1. MoveIt! 関連パッケージのインストール
ROS(melodic) をインストール後、以下のコマンドを実行してください。  
```
$ sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers
$ sudo apt install ros-melodic-industrial-trajectory-filters
$ sudo apt install ros-melodic-moveit
```
上記コマンドを実行後、rosdep を初期化してください。  

## 3.2. ROS1-Bridge のインストール
以下のコマンドを実行してください。  
```
$ sudo apt install ros-crystal-ros1-bridge
```

# 4. ロボットモデル制御アプリケーションのビルド

## 4.1. ソースの展開
事前にダウンロードした ROS1_OpenManipulatorX_EVK-master.zip を展開します。  
以下のコマンドを実行してください。  
```
$ source /opt/ros/melodic/setup.bash
$ cd ~
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ unzip ~/ROS1_OpenManipulatorX_EVK-master.zip
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```

## 4.2. アプリケーションのビルド
以下のコマンドを実行してください。  
```
$ cd ~/catkin_ws/
$ catkin_make
```

# 5.Windows 側環境構築手順

## 5.1. プロジェクトのインポート
e2studio を起動し、ファイル -> インポート から「ROS2_RZA2M_MoveIt1-master.zip」をインポートします。その際、 "ROS2_RZA2M_MoveIt1-master.zip_expanded\ROS2_RZA2M_MoveIt1-master\demos\renesas\rza2m-ebk\e2studio" プロジェクトのみにチェックを入れてください。  

**「ROS2_RZA2M_MoveIt1-master.zip」はC:\に設置してください。階層が深いとe2studioへ正常にインポートされないことがあります。**


## 5.2. プロジェクトの設定変更とビルド
以下の手順を実行してください。  
(1) プロジェクトエクスプローラを右クリックし、プロパティを選択する。  
(2) C/C++ ビルドの「設定」 → Cross ARM C++ Compiler の Preprosessor を選択する。  
(3) Defined Symbol を以下に変更する。  

```
USING_UXR
USING_FIXED_IPADDR
DDS_TRANSPORT=DDS_TRANSPORT_UDP
CLIENT_STREAMS=CLIENT_STREAMS_RELIABLE
```

変更を保存後、プロジェクトをビルドしてください。  

## 5.3. プロジェクトの書き込み
以下の手順を実行してください。  
(1) マイコンに電源を入れる前に、あらかじめ J-Link と PC を USB 接続しておく。  
(2) マイコンに電源を入れ、e2studio から 実行[R] → デバッグ[D] を選択する。  
* Renesas GDB Hardware Debugging を選択 → OK  
* J-Link ARM を選択 → OK  
* R7S921053 を選択 → OK  

(3) 書き込み完了後、デバッガを終了し、J-Link を取り外す。  

# 6. ロボット制御環境起動手順

## 6.1. Linux 側

**事前に ~/.bashrc で source /opt/ros/\<package\>/setup.bash、または source ~/catkin_ws/devel/setup.bash を呼び出していないことを確認してください。**  

端末を4つ（端末A～D）開き、以下の順に実行してください。  

[端末A]  
```
$ MicroXRCEAgent udp --port 2020 --discovery
```

[端末B]  
```
$ source /opt/ros/crystal/setup.bash
$ source /opt/ros/melodic/setup.bash
$ ros2 run ros1_bridge dynamic_bridge
```

[端末C]  
```
$ source /opt/ros/melodic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch open_manipulator_evk_moveit demo.launch
```

[端末D]  
Gazebo との連携が必要な場合、以下のコマンドを実行してください。  
```
$ source /opt/ros/melodic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch open_manipulator_evk_gazebo open_manipulator_gazebo.launch
```

<br>
端末B 起動時、以下のエラーメッセージが繰り返し表示されますが、動作には問題ありません。  

```
[ERROR] [1567752565.159206520]: [registerPublisher] Failed to contact master at [localhost:11311]. Retrying…
```

```
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
```

## 6.2. SEMB-1451/1452 側
電源を入れてください。  

## 6.3. 起動確認方法  
Linux 側の端末A に、下記のように表示されます。  

```
[1565169949.224506] info     | DiscoveryServerLinux.cpp | init                     | running...             | Port: 7400
[1565169949.224969] info     | UDPServerLinux.cpp | init                     | running...             | port: 2020
[1565170101.307103] info     | Root.cpp           | create_client            | create                 | client_key: 0xAAAABBBB, session_id: 0x81
[1565170101.307270] info     | UDPServerBase.cpp  | on_create_client         | session established    | client_key: 0xAAAABBBB, address: 192.168.2.52:8983  → client_key: 0xAAAABBBB の publisher が接続されています。
[1565170106.425678] info     | Root.cpp           | create_client            | create                 | client_key: 0xCCCCDDDD, session_id: 0x81
[1565170106.425794] info     | UDPServerBase.cpp  | on_create_client         | session established    | client_key: 0xCCCCDDDD, address: 192.168.2.52:33168　→ client_key: 0xCCCCDDDD の subscriber が接続されています。
```
また、新たに端末を立ち上げ、以下のように入力すると ROS 1<=> ROS 2ブリッジの動作が確認できます。  

```
$ source /opt/ros/crystal/setup.bash
$ ros2 topic list
/parameter_events
/open_manipulator_evk/demo_arm_srv_pos_down  → subscriberのトピック名
/open_manipulator_evk/demo_arm_srv_pos_up  → publisherのトピック名
/rosout
```

# 7. 操作方法

## 7.1. MoveIt! の操作方法  
端末C で起動した MoveIt! を使用してアームの姿勢を制御します。  
(1) MotionPlanning - Planning タブの中にある Allow Approx IK Solutions にチェックを入れる。  
 → モデル上部に表示されている水色の球体 (interactive marker) をドラッグすることで姿勢変更が可能    
(2) モデルの姿勢が決まったら、左側の Plan and Execute ボタンを押下する。  
(3) 全ての通信が正常に動作していれば、Robot Arm が指定した位置へ動作する。  

※ 水色の球体 (interactive marker) を、**ロボットを設置している面より低い位置に設定しないでください。**  
※ あまり極端に動かした場合、**MoveIt! が例外を出して終了する、またはハングアップする場合があります。**  
終了、またはハングアップした場合は、以下の手順で全ての端末を再起動してください。  

(1) Ctrl+C を押して、端末D のプロセスを完全に終了させる。  
(2) 各端末を 6-1. 項の手順で再度実行する。  

# 8 配信されるトピックについて

## 8.1. システム構成
本システムの構成は以下の通りです。 

```
                   +-----------------+            +-----------------+
                   |     MovevIt!    |            |     Gazebo      |
                   +-----------------+            +-----------------+
                       |        ↑                          ↑
                    1. |        | 2.                       |
                       ↓        |                          |
               +------------------------+         7.       |
               | arm_controller_adaptor |------------------+
               +------------------------+
                       |        ↑
                    3. |        | 4.
                       ↓        |
                  +-----------------+
                  |   ros1_bridge   |
                  +-----------------+
                       |        ↑
                    5. |        | 6.
                       ↓        |
                  +-----------------+              +-----------------+
                  |  XRCE DDS Agent |              |    Robot Arm    |
                  +-----------------+              +-----------------+
                       |        ↑                      ↑         |
                       |        |                      |         |
                       ↓        |                      |         ↓
                  +-----------------+              +-----------------+
                  | XRCE DDS Client |←------------→|   ROS 2 リスナー |
                  +-----------------+              +-----------------+

```
</br>

## 8.2. トピックの詳細
前項のシステム構成図にて記載のあった 1. ～ 7. で配信されているトピックについて説明します。  

| No. | トピック名 | メッセージ型 | 内容 |
|:----------:|:-----------|:------------|:------------|
| 1. | /open_manipulator_evk<br>/open_manipulator_evk_joint_controller<br>/follow_joint_trajectory/goal | control_msgs<br>/FollowJointTrajectoryActionGoal | MoveIt! から発行された経路計画上の各サーボ状態(位置、速度、加速度、電流)。 |  
| 2. |  /open_manipulator_evk<br>/open_manipulator_evk_joint_controller<br>/follow_joint_trajectory/feedback | control_msgs<br>/FollowJointTrajectoryActionFeedback  | ロボットアームから発行された各サーボ状態。 |  
| 3. | /open_manipulator_evk<br>/demo_arm_srv_pos_down | std_msgs/Float64MultiArray | MoveIt! から発行された経路計画上の各サーボ状態。control_msgs/FollowJointTrajectoryActionGoal から std_msgs/Float64MultiArray 型のメッセージに変換されている。MoveIt! の軌道計画上の時刻が付与されている。 |  
| 4. | /open_manipulator_evk<br>/demo_arm_srv_pos_up | std_msgs/Float64MultiArray | 以下のNo.6がros1_bridgeによって転送されたもの。 |  
| 5. | /open_manipulator_evk<br>/demo_arm_srv_pos_down | std_msgs/Float64MultiArray | 上記No.3がros1_bridgeによって転送されたもの。(ROS 2 topic)|  
| 6. | /open_manipulator_evk<br>/demo_arm_srv_pos_up | std_msgs/Float64MultiArray | ロボットアームから発行された各サーボ状態。 (ROS 2 topic)|  
| 7. | /open_manipulator_evk<br>/joint**_position/command | std_msgs/Float64 | 上記No.4がarm_controller_adaptorによって転送された、Gazeboに対する各サポート状態。|  




<br>

## 8.3. std_msgs/Float64MultiArray のデータ構造
std_msgs/Float64MultiArray のデータ構造は以下の通りです。  

| IDX | 内容 | 備考 |
|:----------:|:-----------|:------------|
| 0   | サーボ位置の時刻        | 単位: ナノ秒 MoveIt!の軌道計画上の時刻|  
| 1   | サーボ1の角座標        | 単位: radian|  
| ... | ...        | ...|  
| 5   | サーボ5の角座標        | 単位: radian|  
| 6   | サーボ1の角速度        | 単位: radian/s|  
| ... | ...        | ...|  
| 10  | サーボ5の角速度        | 単位: radian/s|  
| 11  | サーボ1の角加速度        | 単位: radian/s^2|  
| ... | ...        | ...|  
| 15  | サーボ5の角加速度        | 単位: radian/s^2|  
| 16  | サーボ1の電流量        | 単位: パーセント（今回は未使用）|  
| ... | ...        | ...|  
| 20  | サーボ5の電流量        | 単位: パーセント（今回は未使用）|  

</br>

# 制限事項

* Gripper の MoveIt! からの制御には未対応  
	現状では arm_controller_adaptor から Robot Arm には常に初期値が出力されます。

----