# 🐍 Serpens

> Snake-shaped pet robot R&D project exploring mechanical design, perception, simulation, embedded control, and Physical AI.

Serpensは、家庭内で人や環境と関わることを目指して開発している、多関節の蛇型ペットロボットです。

単に移動するロボットではなく、

- 周囲を認識する
- 人の存在や距離に反応する
- 状態に応じて行動を変える
- 蛇らしい身体表現を行う
- 将来的に家庭内AIと連携する

といった、人と同じ空間で存在感を持つロボットを目指しています。

現在は **実機完成前のR&D段階** で、CAD、制御ソフトウェア、知覚、シミュレーション、基板設計を並行して進めています。

---

## 🎥 Development Overview

▶️ YouTube  
https://youtu.be/guUCysoadRg

この動画は、現在のSerpens開発状況を

- CAD
- MuJoCoシミュレーション
- 頭部センサ基板の3Dレンダリング

を通してまとめたものです。

> ⚠️ 現時点では実機動画ではありません。  
> 動作映像はシミュレーションであり、頭部センサ基板も設計・3Dレンダリング段階です。

---

## 🧩 Current Status

| Area | Status |
|---|---|
| Mechanical / CAD | 🟡 Development |
| Motion / Gait | ✅ Software verified |
| 2D Simulation | ✅ Working |
| MuJoCo Simulation | 🟡 Development / evaluation |
| Perception | ✅ Software / simulated verification |
| Behavior System | ✅ Software verified |
| GUI | ✅ Working |
| Head Sensor PCB | 🟡 Designed, not manufactured |
| Main PCB | 🔴 Early design stage |
| Servo Hardware | ⚪ Hardware verification pending |
| Full Robot | ⚪ Not assembled yet |

**Hardware verified features: 0**

シミュレーションやソフトウェア上で確認した内容と、実機で確認済みの内容を区別して開発しています。

---

## 🏗 System Overview

```text
Camera / Sensors
       ↓
   Perception
       ↓
   World State
       ↓
Internal State / Behavior
       ↓
 Motion / Gait Generator
       ↓
 Safety / Robot Link
       ↓
 ESP32 / Servo System
```

シミュレーションと実機で同じ上位ロジックを利用できるよう、ハードウェア依存部分を分離した構成を目指しています。

---

## 🐍 Robot Configuration

Serpensは9軸構成を想定しています。

- Body yaw ×6
- Neck pitch ×1
- Head yaw ×1
- Head roll ×1

蛇行移動だけでなく、頭を持ち上げる、周囲を見る、休息姿勢を取るなど、蛇型ロボットならではの身体表現も扱います。

---

## 🧠 Behavior System

Serpensでは、現在の行動選択にLLMを直接使っていません。

人との距離や接触、周囲の状況、サーボ温度などを入力として、内部状態と効用関数、状態機械から行動を決めます。

```text
Perception
↓
Stimulus
↓
Internal State
↓
Utility
↓
State Machine
↓
Motion / Expression
```

内部状態として、

- Curiosity
- Affection
- Stress
- Attention
- Energy

などを持ちます。

例として、

- 人に気づく
- 近づく
- 見る
- 撫でられて反応する
- 疲れたら休む

といった行動を、内部状態に応じて選択します。

この部分では、「AIっぽく見せる」ことよりも、なぜその行動を選んだのかを説明可能な構造にすることを重視しています。

---

## 👁 Perception

現在扱っている知覚系は以下です。

- OpenCV
- ArUco markers
- Homography
- YOLO person detection
- Person tracking
- Snake pose estimation
- Virtual camera for simulation

人検出ではYOLOを使用し、bboxの下辺中央を足元として床座標へ変換します。

ArUco markerからは、Serpens自身の位置・向きを推定します。

また、シミュレーション上の真値を「実カメラで認識できた結果」と混同しないよう、World Stateにはデータの出どころを持たせています。

例：

- `GROUND_TRUTH_SIM`
- `ARUCO`
- `PERSON_DETECTOR`
- `DEAD_RECKONING`
- `UNKNOWN`

---

## 🐍 Motion / Gait

蛇行歩容は、各関節に対して位相差を持つ波を生成して作ります。

基本形：

```text
α(n,t) = A · sin(Ω · n + ω · t) + γ(n)
```

- `A` : 振幅
- `Ω` : 関節間位相差
- `ω` : 時間方向の周波数
- `γ(n)` : 旋回用オフセット

現在は、

- Forward
- Reverse
- Turning
- Rest
- Rear-up
- Head-look
- Stretch
- Relax

などの姿勢・動作を扱っています。

関節角度は、単にソフト上の上限だけでなく、

- Geometry limit
- Mechanical limit
- Operational limit

を分けています。

CAD上の干渉限界と、実際に運用する安全な範囲を区別するためです。

---

## 🧪 Simulation

### 2D Kinematic Simulator

独自の簡易シミュレータを使い、

- 蛇行
- 旋回
- 車輪拘束
- 姿勢
- 人との位置関係
- 歩容による移動量

を検証しています。

車輪付きリンクが横滑りしにくいという拘束から、胴体全体の速度を求めています。

このシミュレータは物理エンジンを使わない簡易モデルなので、結果を実機と同一視しません。

### MuJoCo Simulation

より現実的な物理挙動を確認するため、MuJoCoを使った3D物理シミュレーション環境も導入しています。

主な目的：

- 歩容比較
- 前進量
- トルク
- エネルギー
- 追従誤差
- 摩擦モデル
- 動作候補比較

MuJoCo側では、`config/robot.yaml` からMJCFを生成しています。

シミュレーション結果には必ずsourceを付けます。

例：

- `KINEMATIC_SIM`
- `MUJOCO_SIM`
- `HARDWARE`

簡易シミュレータとMuJoCo、実機の結果を混ぜないことを重視しています。

---

## 🛡 Safety

Serpensでは、実機化を前提として安全系も設計しています。

主な状態：

- `RUN`
- `HOLD`
- `DISABLED`
- `EMERGENCY`

さらに、PC側だけで安全を成立させないため、

- Heartbeat
- Command TTL
- Emergency latch
- Joint limits
- Telemetry monitoring
- Robot-side safety state

などを扱っています。

PCが停止したり、USB通信が途切れたりした場合でも、将来的にロボット側で安全状態へ移行できる構成を目指しています。

ただし、現時点ではハードウェア実機での安全検証は未完了です。

---

## 🔌 Robot Link / Embedded Control

PCとESP32間では、9軸分の角度列を毎回送るのではなく、歩容パラメータや身体姿勢を送る構成を検討しています。

PC側：

```text
Behavior
↓
Motion Parameters
↓
Robot Link
```

Robot側：

```text
ESP32
↓
Gait Generation
↓
Servo Commands
```

これにより、PC通信が途絶えた場合でも機体側で停止処理を行える構成を目指しています。

---

## 🖥 GUI

開発・展示用GUIも作成しています。

主な表示：

- Camera view
- Top-down map
- Internal states
- Current behavior / intention
- Servo status
- Robot safety state

制御ループとGUI描画は分離しています。

- Control loop: 50Hz
- GUI: approximately 10fps

GUI負荷によって制御周期へどの程度影響するかも測定しています。

---

## 📊 Timing Verification

30秒間の制御周期測定例：

| Condition | Average | Worst |
|---|---:|---:|
| GUIなし | 19.99 ms | 20.78 ms |
| GUIあり | 19.99 ms | 58.1 ms |
| GUI + GIF recording | 20.01 ms | 64.7 ms |

PythonのGILや描画処理による遅延も考慮しながら、制御系とUIを分離しています。

---

## 🧠 AI-assisted Engineering

Serpensの開発では、AIエージェントも活用しています。

AIには、

- 調査
- 実装支援
- シミュレーション
- 設計案比較
- テスト作成
- ドキュメント整理

などを任せています。

ただし、設計方針や安全判断、ハードウェア仕様、検証結果の解釈は、人間側で確認することを重視しています。

目指しているのは、単にAIにコードを書かせるのではなく、

```text
Requirement
↓
Design
↓
Simulation
↓
Experiment
↓
Evidence
↓
Decision
```

というエンジニアリングプロセス全体をAIで支援することです。

---

## ⚙️ Technologies

### Robotics / Control

- Python
- Servo Control
- State Machine
- Gait Generation
- Robot Safety Logic
- Perception
- Computer Vision

### Simulation

- Custom Kinematic Simulator
- MuJoCo

### Vision

- OpenCV
- ArUco
- YOLO

### Hardware / Electronics

- ESP32-S3
- Feetech STS3215
- KiCad
- Sensor PCB Design
- Serial Communication

### Mechanical

- Fusion 360
- Multi-joint Mechanism Design
- Interference Verification

### Development

- Git / GitHub
- pytest
- YAML-based configuration
- AI Agents

---

## 🔬 Verification Policy

Serpensでは、「確認済み」という言葉を曖昧に使わないため、検証状態を分けています。

- `SIMULATED`
- `SOFTWARE_VERIFIED`
- `HARDWARE_UNVERIFIED`
- `HARDWARE_VERIFIED`

現在、

**HARDWARE_VERIFIED = 0**

です。

シミュレーション上で成功していても、実機で確認していない場合は実機確認済みとは扱いません。

これはSerpensで特に大切にしている開発ルールの一つです。

---

## 🧪 Testing

主要な機能は、実機がなくても確認できるようにしています。

例：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -r requirements-nodeps.txt
.\.venv\Scripts\python.exe -m pytest
```

主なテスト対象：

- Motion
- Behavior
- Simulation
- Perception
- Safety
- Communication protocol
- Fault injection
- Timing
- Robot link

---

## ▶️ Running the Simulator

シミュレータ：

```powershell
.\.venv\Scripts\python.exe -m serpens.app --sim
```

デモ：

```powershell
.\.venv\Scripts\python.exe -m serpens.app --sim --demo
```

Webカメラ + シミュレーション：

```powershell
.\.venv\Scripts\python.exe -m serpens.app --sim --camera 0
```

MuJoCo gait evaluation：

```powershell
.\.venv\Scripts\python.exe tools\gait_sweep_mujoco.py
```

---

## 🧰 Development Tools

主な確認ツール：

```text
tools/
├─ check_env.py
├─ servo_setup.py
├─ plot_motion.py
├─ sim_view.py
├─ perception_demo.py
├─ calibrate_floor.py
├─ perception_live.py
├─ behavior_demo.py
├─ gait_sweep.py
├─ gait_sweep_mujoco.py
├─ loop_timing.py
├─ localization_sim.py
├─ floorwatch_eval.py
├─ motion_quality.py
├─ vision_check.py
└─ link_check.py
```

---

## 🗂 Repository Structure

```text
serpens/
├─ config/
│  └─ robot.yaml
│
├─ serpens/
│  ├─ behavior/
│  ├─ gui/
│  ├─ hw/
│  ├─ link/
│  ├─ localization/
│  ├─ motion/
│  ├─ perception/
│  ├─ sim/
│  └─ world_state.py
│
├─ simulation/
│  └─ mujoco/
│
├─ firmware/
│  └─ serpens_esp32/
│
├─ tools/
├─ tests/
├─ docs/
└─ README.md
```

---

## 🚧 Current Limitations

現在は以下が未完了です。

- Full robot assembly
- Hardware servo verification
- Head sensor PCB manufacturing
- Main control board completion
- Real camera → robot autonomous control integration
- Physical safety verification
- Real-world Sim-to-Real calibration
- Full household autonomous navigation
- Home AI integration

これらを完成済みとして扱わず、現在の開発状態をそのまま記録しています。

---

## 🛠 Next Steps

今後の主な予定：

1. Head Sensor PCB manufacturing
2. Main control board design
3. Servo / ESP32 hardware verification
4. Partial mechanism assembly
5. Real camera perception
6. Hardware motion testing
7. Sim-to-Real calibration
8. Full robot assembly
9. Human interaction testing
10. Home AI integration

---

## 💡 Design Philosophy

Serpensでは、性能だけでなく

> 「人が一緒に暮らしたいと思えるか」

を大切にしています。

蛇らしい細長い身体や多関節構造を活かしながら、頭部や動きには少し愛嬌を持たせたいと考えています。

機械としての機能性と、ペットロボットとしての親しみやすさを両立することが目標です。

また、いきなり完成形を作るのではなく、

```text
CAD
↓
Simulation
↓
Small Prototype
↓
Physical Test
↓
Design Update
```

という小さな検証ループを回しながら開発を進めています。

---

## 📚 Detailed Documentation

より詳しい内容は `docs/` にまとめています。

例：

- Servo registers
- Link protocol
- Verification status
- Phase acceptance criteria
- Human perception pilot
- Task / Event API
- Safety considerations
- Simulation results

---

## 📌 Project Status

Serpens is currently an **active R&D project**.

まだ完成したロボットではありませんが、

- Mechanical Design
- Electronics
- Perception
- Simulation
- Control
- Safety
- AI-assisted Engineering

を一つのロボット開発としてつなげながら、段階的に実機化を進めています。
