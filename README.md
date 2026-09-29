# d1t-teleop

[GELLO](https://github.com/wuphilipp/gello_software) を使って Unitree D1-T をテレオペするためのレポジトリ。

> 状態: **環境構築済み・コードの雛形あり・実機未接続**（2026-09-29 時点）。次にやることは [明日の作業](#明日の作業) を参照。

---

## 環境構築
```bash
git submodule update --init --recursive
uv sync
```

実行は基本 `uv run python ...`。

- **Python は 3.10 固定**。unitree_sdk2py が要求する `cyclonedds==0.10.2` のビルド済み wheel（CycloneDDS 本体同梱）が cp310 までしか無いため。3.11 以上だと CycloneDDS (C) を自前ビルドして `CYCLONEDDS_HOME` を指定する必要がある
- gello / unitree_sdk2py / DynamixelSDK は submodule を `[tool.uv.sources]` で editable 参照している
- gello_software の `requirements.txt` は UR / xArm / RealSense 等まで入る重い構成なので入れていない。必要になったら `pyproject.toml` に個別に足す

## ファイル構成

```
d1t_teleop/
  msg.py         D1 の DDS 型（ArmString_, PubServoInfo_）を Python に移植 ※型は要確認
  config.py      リーダー(GELLO)のキャリブ値、D1 の関節リミット・グリッパ範囲 ※TODO多数
  d1t_robot.py   D1TRobot: GELLO の Robot プロトコル実装（dry-run 既定）
scripts/
  probe_d1.py    読み取り専用。D1 の状態トピックの受信周期と値を表示
  teleop.py      GELLO → D1 テレオペ（dry-run 既定、--live で実送信）
third_party/
  gello_software/        submodule
  unitree_sdk2_python/   submodule
vendor/          Unitree のソース置き場（git管理外）
```

コード中の `TODO(verify)` は、D1 の SDK / `marm_code` を見て確定させる箇所。

---

## 構成

```
[自作GELLOリーダー (Dynamixel, D1-Tの2/3スケール)]
        │ USB (U2D2)
        ▼
  GelloAgent (gello_software, Python)
        │ ZMQ
        ▼
  D1TRobot (このレポジトリで実装, Python)
        │ DDS (CycloneDDS / unitree_sdk2_python)  ── Ethernet
        ▼
  D1-T 内蔵Linuxボード (marm_* サービス) → サーボ
```

- 2/3スケールでも問題なし（GELLOは関節角をそのまま写すだけ）。必要なのは `joint_offsets` / `joint_signs` のキャリブレーションのみ
- グリッパ: D1-T純正（J6）。GELLOのトリガー値 0〜1 を J6 の角度範囲に線形マッピングする

## 決定事項

| 項目 | 決定 | 理由 |
|---|---|---|
| ROS2 | 使わない | まずシンプルに。必要になったら後で被せる |
| シミュレータ | 使わない | 不要 |
| 言語 | Python のみ | GELLO本体は純Python（C++はROS2版Franka用のみで無関係） |
| D1との通信 | unitree_sdk2_python (cyclonedds) で DDS を直接 publish/subscribe | D1のメッセージ型(IDL)だけPythonに移植すれば済む |
| 実装方針 | GELLOの `Robot` プロトコル（`gello/robots/robot.py`）を満たす `D1TRobot` を書く | 通信方式を変えても中身の差し替えだけで済む |

`Robot` プロトコルで必要なメソッド: `num_dofs()`, `get_joint_state()`（rad, グリッパは0〜1）, `command_joint_state(q)`, `get_observations()`（`joint_positions`, `joint_velocities`, `ee_pos_quat`, `gripper_position`）。

## D1-T について分かっていること

### ハードウェア
- 6DoF + グリッパ(J6) の計7サーボ
- インタフェース: RJ45 (DDS通信), Type-C (シリアルデバッグ), DC電源 24V
- **内蔵Linuxボードあり**。SSHで入れて、ドライバソース `~/marm_code` が載っており、その場で `make` して作り直せる

### 内部サービス（systemd）

`marm_communication` / `marm_control` / `marm_controller` / `marm_subscripber`

公式ドキュメントのトピック定義から推定した流れ:

```
PC ──rt/arm_Command (ArmString_, JSON)──▶ marm_communication_node   JSON解釈
                                               │ set_servo_angle_control / arm_zero / set_servo_dumping
                                               ▼
                                          marm_control_node          補間・軌道生成（推定）
                                               │ set_servo_angle (SetServoAngle_)
                                               ▼
                                          marm_controller_node       サーボバス駆動
                                               │ current_servo_angle (PubServoInfo_)
                                               ▼
PC ◀──rt/arm_Feedback (ArmString_, JSON)── （フィードバック）
```

- `marm_subscripber` の役割は不明
- 角度の単位は **度**（GELLOはrad → 変換が必要）

### 通信の仕様（公式ドキュメントより）

- 指令: `rt/arm_Command` に `ArmString_` の JSON。例: `{"seq":4,"address":1,"funcode":7}`（姿勢ゼロ点）
- 状態(JSON): `arm_Feedback`, seq=10 固定, **10Hz**。address + funcode で種類を区別
- 状態(生): `current_servo_angle` に `PubServoInfo_`（`servo0_data`〜`servo6_data`）。**周期は未確認**
- NIC指定: `ChannelFactory::Instance()->Init(0, "eth0")`（Pythonなら `ChannelFactoryInitialize(0, "<NIC名>")`）
- 公式のテレオペ構成あり（ハンドデバイス付きの「acquisition arm」側は既定サービスを止めて専用プログラムを動かす）

### 未解決・要注意

- ⚠️ **指令を何Hzまで受け付けるか不明**（最重要。GELLOは約100Hzで指令を出す）
- ⚠️ `current_servo_angle` が10Hzより速いか不明
- ⚠️ トピック名の食い違い: サンプルは `"arm_Feedback"` を購読、ドライバは `"rt/arm_Feedback"` を publish。
  unitree_sdk2py（Python）は `rt/` を自動で付けないことをループバックで確認済み。C++ SDK 側の挙動は未確認なので、`probe_d1.py` で両方購読して確かめる
- ⚠️ 全関節指令の JSON 形式（funcode / mode / フィールド名）は推測で書いている → `d1t_robot.py` の `_build_command()`
- 販売店スペックには「SDKで1kHz更新」「遅延15ms以下」とあるが真偽不明

## 制御周期が足りなかった場合の対策（軽い順）

1. **ソフトで緩和**: 先読み指令（リーダー速度から約100ms先を予測）、delay_ms 調整、ローパス/デッドバンド
2. **10Hzで割り切る**: Diffusion Policy 等は10Hzの事例が多い。GELLOの指令値（100Hz）を action として記録する
3. **JSON層をバイパス**（本命）: `set_servo_angle` に `SetServoAngle_` を直接 publish。`marm_control_node` も同じトピックに出すので競合に注意。ソースがあるので10Hzの原因（ループ周期かサーボバス速度か）を特定できる
4. フォロワーを変える（最終手段。リーダーはD1-T専用なので避けたい）

## ロードマップ

| Step | 内容 | 完了条件 |
|---|---|---|
| 0 | ✅ レポジトリ構成・環境構築・雛形（`D1TRobot`, `probe_d1.py`, `teleop.py`）。ループバックで DDS の送受信・変換・クリップを確認済み | — |
| 1 | **D1調査・通信検証**: `marm_code` 読解、PythonからDDSで読み書き、周期・遅延の実測 | 指令周期の上限、フィードバック周期、使う経路（JSON or 直接）が決まる |
| 2 | **リーダーのキャリブレーション**: `scripts/gello_get_offset.py`、7軸目はグリッパ | リーダー読み値がD1と同じ座標系・rad で出る |
| 3 | **`D1TRobot` 実装**: rad↔deg、関節リミットでクリップ、1ステップ最大変化量制限、起動時のゆっくり同期、切断時停止 | dry-run（publishせずログのみ）→ 実機追従 |
| 4 | データ収集: カメラ、保存形式（ACT / LeRobot 等） | — |

---

## 明日の作業

### 1. D1-T にログインする

```bash
# PCのNIC名を確認（D1-Tを繋いだ方）
ip a

# D1-TのIPを探す（Unitree製品は 192.168.123.x が多いが未確認）
sudo nmap -sn 192.168.123.0/24

ssh <user>@<D1-TのIP>
```

入れない場合は Type-C でシリアルコンソール:

```bash
sudo dmesg | tail          # /dev/ttyUSB* or /dev/ttyACM* を確認
screen /dev/ttyUSB0 115200
```

### 2. バックアップ（書き換える前に必ず）

```bash
# D1-T上で
tar czf ~/marm_code_backup_$(date +%Y%m%d).tar.gz ~/marm_code
systemctl status marm_communication marm_control marm_controller marm_subscripber
```

### 3. ソースをPCにコピー

```bash
# PC上で
mkdir -p vendor
scp -r <user>@<D1-TのIP>:~/marm_code vendor/
scp <user>@<D1-TのIP>:~/marm_code_backup_*.tar.gz vendor/
```

`vendor/` は Unitree のソースなので **gitにコミットしない**（`.gitignore` に追加済み）。
SDKのzip（公式ガイド: https://support.unitree.com/home/zh/developer/D1Arm_services ）もあれば `vendor/` に置く。

### 4. ソースで確認すること

- [ ] `marm_controller_node.cpp` のループ周期（`sleep` / `usleep` / Hz の値）とサーボバスの種類・ボーレート
- [ ] `current_servo_angle` の publish 周期
- [ ] `marm_control_node` が `set_servo_angle` を常時 publish しているか（直接指令と競合するか）
- [ ] JSON の funcode 一覧（有効化、全関節指令、delay_ms、グリッパ）
- [ ] IDL / 生成 `.hpp`: `ArmString_`, `PubServoInfo_`, `SetServoAngle_`, `SetServoDumping_` のフィールド定義（Python移植用）

目安のgrep:

```bash
grep -rnE "sleep|Hz|rate|baud|ttyS|ttyUSB|ttyACM" vendor/marm_code/src
find vendor/marm_code -name "*.idl" -o -name "*_.hpp"
```

IDL の型が `d1t_teleop/msg.py` と違ったら直す（特に `PubServoInfo_` が float32 か double か）。型が合わないと DDS は**エラーを出さずに何も届かない**。

### 5. PCから状態を読む（読み取り専用なので安全）

```bash
uv run python scripts/probe_d1.py --nic <D1を繋いだNIC名>
```

- [ ] `current_servo_angle` が届くか、何Hzか（← 10Hzより速ければ嬉しい）
- [ ] `arm_Feedback` と `rt/arm_Feedback` のどちらに届くか
- [ ] 何も届かない → NIC名 / IPの同一サブネット / `msg.py` の型 を疑う

### 6. GELLO リーダーのキャリブレーション（D1 と独立に進められる）

1. D1-T の既知の姿勢（全関節0°など）と同じ姿勢に GELLO を置く
2. オフセット計算:
   ```bash
   ls /dev/serial/by-id/
   uv run python third_party/gello_software/scripts/gello_get_offset.py \
       --port /dev/serial/by-id/<...> \
       --start-joints 0 0 0 0 0 0 \
       --joint-signs 1 1 1 1 1 1
   ```
   `joint_signs` は各関節を動かして D1 と回転方向が逆なら -1
3. 出力を `d1t_teleop/config.py` の `LEADER_CONFIG` に書く（グリッパの開/閉の角度も）
4. リーダー単体で確認: `uv run python scripts/teleop.py --gello-port /dev/serial/by-id/<...>`
   → 表示される角度が D1 の関節角の定義と一致するか

### 7. 指令の検証（Claudeと一緒に）

`_build_command()` の JSON 形式を SDK ドキュメントで確定させてから:

- [ ] dry-run: `uv run python scripts/teleop.py --gello-port ... --nic ... --use-d1`（送信はしない、出す予定の JSON を表示）
- [ ] 実送信は小さい角度・1関節から。`rt/arm_Command`（JSON）と `set_servo_angle`（直接）を 10 / 30 / 50 / 100Hz で送り、追従性・遅延を比較（計測スクリプトは未作成）

⚠️ 実機を動かすときは、小さい角度・1関節ずつ・非常停止（電源）に手が届く状態で。

---

## 参考

- GELLO: https://github.com/wuphilipp/gello_software
- unitree_sdk2_python: https://github.com/unitreerobotics/unitree_sdk2_python
- D1 SDK を CMake パッケージ化した非公式レポ: https://github.com/2Nitrogen/unitree_d1_sdk_extension
- D1 公式開発ガイド（中国語）: https://support.unitree.com/home/zh/developer/D1Arm_services
