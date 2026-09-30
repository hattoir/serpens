# DECISIONS — serpens

<!-- 将来の自分と将来の Claude が「なぜこうなっているのか」を再構築できるように書く。
     新しい判断は一番上に足す（新しい順）。 -->

## テンプレート

```
## YYYY-MM-DD — <決めたこと>

**Decision**: 何を決めたか
**Why**: なぜそれを選んだか
**Alternatives**: 他に何を検討したか
**Trade-offs**: 何を捨てたか / どういう時に間違いになるか
**Context**: その時点で分かっていたこと
```

---

## 2026-09-29 — MQTT 残りの 3 件: Endpoint(PahoBroker) の通常構築 / 切断中の publish / close の TOCTOU（DEC-SERPENS-0001 の続き）

**Decision**:
A. `PahoBroker(autoconnect=False)` → `Endpoint.__post_init__` が will・購読・announce を登録してから `broker.start()`。
   `set_will` は接続前なら受け付け、接続後は例外（黙って無視しない）。接続は `connect_async`（ブローカーが落ちていても生成で例外にしない）。
B. 切れている間の publish は例外にしない。**retain 付き（safety_state）は捨てる**、retain 無しは paho の QoS1 待ち行列に残す。
C. `Broker.enqueue()`（積む）と待つ関数を分けた。`Endpoint` は safety_state の「出すか決める → 積む」を
   `_safety_lock`（RLock）の中で、**PUBACK を待つのはロックの外**で行う。close は closed を立てて OFFLINE を積むまでを同じロックの中で行う。
   close の後の `safety_state` は mode と錠は更新するが送らない。

**Why**:
A. 以前は `Endpoint(PahoBroker)` が `set_will` の例外で組めず、試験は `__new__` で回避していた（本番の組み方が試されていない）。
B. paho は切断中の publish で `RuntimeError` を投げ、主ループの `tick` まで上がっていた（ブローカーが落ちると機体側のループが止まる）。
   溜めるだけにすると、再接続時に announce（今の値）の**後から**古い値が届く。変異試験で
   `EMERGENCY_LATCHED → DRIVING → EMERGENCY_LATCHED` を観測した（Home AI には緊急停止中の機体が一瞬「走行中」に見える）。
   safety_state は「最後の値」だけが意味を持ち、再接続時の announce が今の値を出すので、溜める理由が無い。
C. closed の判定と積む操作の間に close が割り込むと、OFFLINE の後ろに古い状態が並んだ。

**Alternatives**: 送信全体（PUBACK 待ちを含む）を 1 つのロックで囲む → 却下。主スレッドが待つ間、ネットワークスレッドの
announce がロック待ちで固まり、DEC-SERPENS-0001 の不具合（keepalive 切れ → 偽の LWT）が戻る。
切断中の retain を paho の待ち行列から後で消す → paho の内部に触れる。送る前に接続を確かめる方が小さい。

**Trade-offs**: 「接続を確かめる → 積む」の間に切れた場合は、古い safety_state が 1 件だけ paho に残りうる
（再接続後に announce の後から届くが、その後の周期送信が今の値で上書きする）。切断中の retain 無しイベントは
paho の待ち行列に上限なく溜まる（今は周期的な retain 無しイベントが無いので実害なし。battery を周期化するなら上限を置く）。

**Context**: 回帰試験は `test_endpoint_builds_on_paho_normally_and_announces_on_first_connect`（A）、
`test_publishing_while_the_broker_is_down_does_not_raise_and_reconnect_restores_now`（B）、
`test_offline_is_last_even_when_close_races_an_announce_on_another_thread` と
`test_safety_state_after_close_does_not_overwrite_offline`（C）。どれも修正を外すと落ちることを確かめた。

---

## 2026-09-29 — DEC-SERPENS-0001 MQTT safety_state retain race fix（PahoBroker はネットワークスレッド上では PUBACK を待たない）

**Decision**: `PahoBroker.publish` は paho のネットワークスレッド（on_connect / on_message のコールバック）から
呼ばれたときだけ `wait_for_publish` を飛ばす。`on_connect` を接続後に登録した場合は、その場で 1 回呼ぶ。
`Endpoint.close()` は OFFLINE を出す**前**に `closed` を立てて状態を捨て、以後 `announce` / `tick` は何も出さない。
`tests/test_mqtt_live.py` の購読者は client_id を別々にした。

**Why**: 全体実行で `test_graceful_close_publishes_offline_immediately_and_reconnect_overwrites` が 1 回落ちた
（遅れて購読した側に OFFLINE が届いた）。原因は 2 つ重なっていた。
(1) **実装のバグ**: 再接続時の `announce` と Task 受信時の `task_status` はネットワークスレッド上で publish する。
そこで PUBACK を待つと、PUBACK を読むのがそのスレッド自身なので必ず 2 s のタイムアウトまで固まる。
その間は送信も PINGREQ も止まる。実ブローカーでは、生きている機体が `exceeded timeout` で切られて LWT の OFFLINE が出た。
再接続すると announce でまた固まり、DISARMED が一度も届かないまま OFFLINE → 再接続 → OFFLINE を繰り返した
（CPU 負荷なしでも再現）。stop Task を受けたときも、中止通知 1 件ごとに 2 s ずつ処理が遅れる。
on_connect を接続後に登録すると、CONNACK が先に処理されたときは初回の announce が抜ける。
そのため、この固まりはスケジューリング次第で出たり出なかったりした。
(2) **試験のバグ**: `_home` が 2 つとも client_id `home-test` だった。ブローカーが古い方を蹴り、paho の
自動再接続で互いを蹴り合っていた（mosquitto のログに `session taken over` が約 1 s ごと）。そのため再接続のたびに retain が届き直し、
「DISARMED を retain で受けた」(153 行) が、最初の購読で OFFLINE を受けた後の再購読で成立していた。
2 つ目を直すと、1 つ目が起きたときは 153 行で正しく落ちる（アサーションは緩めていない）。

**Alternatives**:
(1) テストに待ちを足す / 1 回落ちを flaky 扱いにする → 却下。生きている機体が OFFLINE と報告され続ける実在の不具合。
(2) announce を別スレッドへ逃がす → publish 側で直す方が、on_message の task_status も同時に直る。
(3) Endpoint の safety 送信全体をロックで囲む → 主スレッドが PUBACK を待つ間ネットワークスレッドが止まり、同じ問題が再発する。

**Trade-offs**: ネットワークスレッドからの publish は、戻った時点でブローカー受理が保証されない。
送信順は paho の送信待ち行列の順で保たれ、QoS1 の再送もある。
close と、別スレッドの announce / tick の間の判定と送信の隙間（TOCTOU）は、`closed` だけでは完全には閉じていない（狭い窓が残る）。 → 同日の続き（上）で解消。

**Context**: 回帰試験は `test_reconnect_overwrites_lwt_from_the_network_thread_without_stalling_it`（実ブローカー）と
`test_nothing_overwrites_the_offline_after_a_graceful_close`（Loopback）。どちらも修正前のコードで落ちることを確認した
（announce が 2.03 s 固まる／close 後の再接続で BOOT が OFFLINE を上書き）。
修正後: CPU 32 並列の負荷下で `tests/test_mqtt_live.py` を 15 回連続 → 15/15 通過。全体 466 passed。

---

## 2026-09-29 — 頭内取り込み機構（スコップ＋フタ）は第 2 段階の製品機能。今回の作業はその成立性調査（User 決定）

**Decision**: PRODUCT.md §2 のとおり、「保持」を「守る（MVP）→ 頭内に閉じ込める（第 2 段階）」に更新した。やらないのは「口でくわえて運ぶ」「腹まで運ぶ」。
頭内取り込み機構は**第 2 段階の製品機能**で、MuJoCo の作業（`simulation/scoop/`、`config/scoop.yaml`）はその**成立性調査**。
閉じ込めた物が出ない・小部品が外れない・ボタン電池を溜める危険は、**機構設計と ST 基準で扱う**（シミュレーションの対象外、先送り）。

**Why**: 床見守りの中心目的（小物誤飲）に対し、見つけるだけでなく守る（閉じ込める）まで進めたい。ただし実物の試験片の結果が出るまで、シミュレーションは「候補」の根拠にしない。

**Findings（2026-09-29 の探索。`simulation/results/scoop_exploration_2026-09-29.md`）**: 指示書の範囲（先端厚 0.4〜1.0 mm、円柱の縁は直角、一定速度で押しつける）では、
**全条件で成功 0（1,728 回）。物は摩擦・速度・先端の浮きに関係なく、頭と同じ速さで前へ運ばれる**（構造的。バグではない: 傾斜板の保持条件 tan α ≤ μ は解析と一致）。
縁の丸み・前面の向き・先端 0.02 mm まで広げても成功 0。先端 ≤ 0.1 mm・丸み ≥ 0.3 mm・低摩擦ではコインが傾斜板に乗るが、先端から約 9 mm の位置で運ばれるだけで空間に入らない。
→ 掃引（N = 10 → N = 30）は、設計変数を広げる方針が決まるまで回していない。

**Alternatives**: 指示書のまま 77,760 回の掃引を回す（全部 0 で比較にならない）。

**Context**: User の指示書（2026-09-29）、`docs/verification_status.md` §4.14。

---

## 2026-09-30 — 頭内取り込みは「巻き込みくちばし」で調べたが、1 円玉・電池・立方体は保持 0（推奨案なし）

**Decision**: 奥ヒンジのフタは幾何的に物を奥へ押せない（依頼側の誤り。User が訂正）ので、ベースラインとして残し、ride / enter の測定にだけ使った。
新しい変種として「巻き込みくちばし」（ヒンジ = ランプ先端の真上、腕長 = H − 0.5mm、180° 回転、閉じ終わりに機械ラッチ）を調べた。
指標を修正した: 乗る = 物の縁が床から 1mm 以上上がった瞬間があった / 入る = 中心が空間の範囲に入った / 保持 = 閉じ終わりから 2 秒後まで空間の中 / 逃げた距離。
**推奨する設計は出さない**（保持は実質ビーズだけ。ビーズも叩き込み）。実物の試験片（短ランプ 8 / 15 + 手で回すピン留めの腕）で予測を確かめる。

**Findings**: `simulation/results/scoop_beak_2026-09-30.md`。144 設計（くちばし）+ 48 設計（ベースライン）× 12 回 = 9,216 回、上位 3 設計を N = 30、感度 19 設計。
1 円玉 0/1,728、立方体 0/1,728、CR2032 7/1,728、ビーズ 146/1,728。上位設計のビーズ 70%（Wilson 90% 55–82%）、飛ばされた分を除くと 50%。
原因（トレース）: 腕先が床に近づくのは x = 0 の一点だけで、先端より前の物には上から当たる → 後ろへ回り込めない。ビーズは叩かれて奥へ飛び、入口が開いたまま跳ね返って出る。
「飛ばされた」率は接触のやわらかさ・時間刻みに依存して収束しない。保持 0（1 円玉・CR2032）は変わらない。

**Ride の定義の読み**: 「前縁」を −x 側の縁と読むと、既知ケース（先端 0.05mm・丸み 0.3 の 1 円玉が 9.1mm まで乗る）で偽になり検証を満たさない。傾斜板に向かう頭側の縁を使った（User に確認したい）。

**Alternatives**: 設計を全組合せ（くちばしだけで約 1.4 万回）で回す → 1 万回未満に絞り（幅 × 側壁を 2 通り、先端厚 0.6 固定）、残りは上位案の感度で見た。結果が悪くても設計を後付けで調整して成功に見せることはしていない。

**Context**: User の依頼（2026-09-30）、`docs/verification_status.md` §4.14、`simulation/scoop/hardware_test_plan.md`。

---

## 2026-09-26 — Floor Watch の inspect は展示の行動を通さず、Task が無ければ止まっている

**Decision**: `FloorWatchExecutor` は Endpoint の Executor として SimSession に `mission` を差し込み、active のあいだ brain.tick を
呼ばない（移動は `Controller.drive_to` の安全だけ通す）。Task が無いあいだは `IdleHold` が歩容を止める。停止位置の散り
（惰行が歩容の位相で 33〜188 mm）と蛇行の横ずれは、開ループの補正ではなく **停止後に頭から地点までの (前方, 左) を測り、
短い前後の微調整（振幅を即 0）と頭ヨーで線を地点に置く** 閉ループで吸収する。Task stop は機体を HOLD にし、再開は API の
operator_resume と機体側の開始操作の両方（人）。

**Why**: 展示の brain は人に反応して勝手に動く（PATROL に戻る）ので、Floor Watch の「Task 以外で動かない」と両立しない。
惰行の開ループ補正は位相依存で当たらなかった（0.5 で 33 mm、1.3 で 188 mm）。線幅 3 mm、床カメラの横視野 ±23 mm に対し
蛇行の横ずれは ±5 cm あり、頭ヨーで向けないと地点が視野に入らない。

**Alternatives**: brain に「TASK」状態を足す（展示の語彙と混ざる）/ 惰行の補正表を位相ごとに持つ（実機で崩れる）/
機体ごと向き直す（時間がかかり、また惰行が乗る）。

**Trade-offs**: 位置合わせの測定は模擬では真値。実機では推定姿勢（σ 数 cm）になるので、JUDGE 後の候補ベースの AIM が主になる。
IdleHold のあいだ呼吸などの表現は止まる（Task 待ちの「生きている感」は後で）。

**Context**: 2026-09-26。KINEMATIC_SIM、合成画像、合成タグ観測。実機の値はゼロ。

## 2026-09-26 — 自己位置の開始条件は σ と局所センサーの健全性で書く（出どころの名前の置換にしない）

**Decision**: フェーズ 3 の推定器（AprilTag + IMU + 歩容オドメトリ）は (x, y, yaw) + 共分散を持ち、`start_blockers` /
`autonomy_blockers` は「σ_xy ≤ 0.15 m、σ_yaw ≤ 0.2 rad、IMU が新しい、歩容の位相が更新されている」で判定する。
`autonomy.pose_sources` に出どころごとの規則を置き、`aruco`（EX-01 の外部カメラ）は検証用として σ なしで残す。
滑りの σ は最後のタグ補正からの距離に比例して育てる（系統誤差。√N にしない）。IMU の絶対 yaw は磁北基準なので
home との差をタグで補正するたびに取り直す。地図は `config/tags.yaml`（座標系 home の定義そのもの）。

**Why**: レビュー「ARUCO → APRILTAG の置換にしない」。名前で判定すると、タグを 1 枚見ただけの後 5 m 走っても
「実観測」のまま走れてしまう。σ で書けば「タグを見ずに走れる距離」が数字（≈ 0.7〜1.1 m）として出る。

**Alternatives**: 粒子フィルタ（多峰性に強いが MVP には過剰）/ 1 歩ごとの独立雑音（σ が √N で育ち、滑りに対して楽観）/
IMU を使わず歩容だけ（向きが育ちすぎる）。

**Trade-offs**: 2D・平面床の前提。滑りの 15% は実測まで仮で、真の滑りが超えると σ は楽観になる（20% で 83%）。
視野 65° の前方カメラは直進中に横のタグを見られない。

**Context**: 2026-09-26。実カメラ・IMU・サーボは無い。フェーズ 2 の f = 1000 px と FOV 65° の矛盾は未解決。

## 2026-09-26 — Floor Watch 画像処理: 基準床を使わない / 測れない高さは null / 鏡面の円盤は危険物側 / Mosquitto はテストが起動する

**Decision**: (1) 「物の無い同じ視点の床」を前提にしない。背景は同じ画像に頑健に当てはめた多項式面、線の基準は較正した面と
物の前後の床上の線。(2) 高さが測れないときは `height_mm: null` + `height_reason`（specular_break / off_line / too_few_rows）。
危険度では測れない = 出っ張り扱い（係数 1.0）。(3) 線の途切れ + 円形 + 直径 5〜25mm は `metal_disc` として critical_kinds に入れ、
確信度に関わらず通知。(4) 斜め照明はあご（床 6mm、前向き）で影は物の奥。撮影順 通常→斜め→線光→全消灯→通常、動いたら撮り直し。
(5) 評価は patrol / inspect の 2 段で信頼区間を付ける。(6) safety_state は 2 s 周期、受け側は自分の時計で 5 s 失効、
正常終了は自分で OFFLINE、operator_resume は MQTT から到達しない。(7) Mosquitto は常駐させず `tests/test_mqtt_live.py` が
127.0.0.1・一時ポート・一時設定でサブプロセス起動する。paho-mqtt は開発用の任意依存。

**Why**: 実機では姿勢誤差が cm 級で基準床は取れない（レビュー 3）。高さ 0 は「平ら = 汚れ」と読まれ、鏡面 = 金属 = ボタン電池
そのものが 0 になるのが最悪（レビュー 1・2）。影の向きは LED の位置で決まる（レビュー 5）。n が小さいときの 90% / 0% は
楽観（レビュー 6）。LWT は異常切断でしか出ない（レビュー 10）。ブローカーを常駐させると「安全経路ではない」通信が
勝手に生き続ける。

**Alternatives**: 局所の中央値フィルタ（物が 280px と大きく、核より大きい物が背景に吸われた）/ 高さ 0 + フラグ（受け側が
フラグを見ないと事故）/ Mosquitto をサービス登録。

**Trade-offs**: 黒い物・透明な物も線が乗らず `metal_disc` になる（安全側の誤報。分類器まで許容）。多項式面は照明むらが
複雑な床では合わない（実写で次数・ぼかしを調整）。合成の 98% / 0% は理想的な影と線での値で、実写の性能ではない。

**Context**: 2026-09-26 のレビュー 1〜11。実写・治具写真・スマホ機種は未回答。Mosquitto の winget 導入は遅い回線で
数十分かかった（mosquitto.org からの直接ダウンロードが約 35 分。インストーラーがサービスを登録するので、導入後に止めて無効化する）。

## 2026-09-25 — Floor Watch: Home AI との境界は Task / Event API だけ。MQTT を関節制御に使わない

**Decision**: Home AI ↔ Serpens は `home/serpens/task`（5 種）と `home/serpens/event/*`（4 種）の JSON だけで接続する。
Task に安全の設定（速度・角度・TTL・トルク）を変えるフィールドは置かない。stop は常に最優先、同じ id は冪等、
`map_version` 違いと実行できない理由（安全ゲート・自己位置なし）は rejected。`safety_state` は retain。
検証器は標準ライブラリだけの最小部分集合（`jsonschema` を依存に足さない）。MQTT クライアントは任意依存（Loopback で試験）。
Serpens 内部の PC → 機体（USB 駆動リンク）はそのまま。5 サーボ MVP でも関節数を前提にしない。首 = J1 + J2 で視線を作る。

**Why**: 決定事項 3・4・14（境界だけ MQTT、PC は Task だけ、別リポジトリ）。安全は機体側で完結し外から触れない、を
API の形で保証する。依存を増やさないのは、`opencv-contrib` の罠（README §1）と同じく環境の再現性のため。

**Alternatives**: (1) 関節角を MQTT で送る（通信遅延で動きが乱れ、安全経路が PC 依存になる）
(2) `jsonschema` / `paho-mqtt` を必須依存にする（フェーズ 5 で必要になったら足す）。

**Trade-offs**: 最小検証器は JSON Schema の全機能を持たない（対応範囲は validate.py に明記）。本物のブローカーとは未疎通。

---

## 2026-09-21 — Human Evaluation を最優先ゲートにし、身体構成の決定は Pilot 後へ

**Decision**: 検証レベルを SOFTWARE_VERIFIED / KINEMATIC_SIM / PHYSICS_SIM / HARDWARE_VERIFIED / HUMAN_EVALUATED の
5 段にし、蛇らしさ・アニマシー・近づきやすさ・愛着・怖さは Simulation だけでは合格判定しない。
6 / 8 / 10 Yaw は同じ Ω を使い回さず「狙い波数 → Ω = w·360/N」で揃え、同等リンク長（B）と同等身体長（A）の両方を作り、
RAW と SPEED_MATCHED の 2 条件を用意した。クリップは匿名 ID で、条件との対応は key JSON にだけ持つ。
内部状態は 6 種（Familiarity / Sleepiness を追加）を共通の式（baseline / rise_tau / decay_tau / saturation / gains）で持ち、
Stress は saturation 0.9 として「飽和で社会的な状態が同時に消える」構造を無くした。Energy（活動）と温度（機械）を分離。

**Why**: 動きの質は最終的に人が決める。Simulation の指標で「良くなった」と言い続けると、展示で初めて破綻する。
6 軸 1 波 519mm vs 8 軸 2 波 199mm のような数値だけで身体を決めると、速度差が蛇らしさの評価を汚染する。
Bug-1 の「飽和 → 回復不能」は gain の値ではなく構造の問題だったので、saturation と property test で押さえた。

**Alternatives**: (1) Simulation の指標を成功指標にする (2) 8 軸を先に採用して CAD を進める（Pilot 前の決定は禁止）
(3) Familiarity を単純なタイマーにする（同一人物の追跡・距離・退避・タッチを入力にした）。

**Trade-offs**: 動画では大きさ・音・接触・距離感は評価できない（VIDEO_HUMAN_EVALUATION として分ける）。
n=8〜12 では小さな差は結論にできない。EXHIBITION profile の体験弧は 3 seeds の模擬で確認したにすぎない。

---

## 2026-09-21 — 動きの語彙（primitives）と文法（grammar, config）を分け、人格を config で変える

**Decision**: しぐさを「語彙」（`primitives.py`。状態を知らない素材）と「文法」（`grammar.py`。config の
`behavior.grammar` に、状態ごとに何をいつ撃つかの表）に分けた。移動の段取りは `locomotion.py`。
Bug-1 は gain を下げるだけでなく、駆け込みを別の出来事（Stress の一段上げ）にした。
Bug-2 の gain_move はレビュー案 0.004 でなく 0.008（stop-and-go の稼働率 0.5 を見込む）。
静止率は「歩容の振幅がゼロ」で定義（首マーカの速さだと呼吸の位相波を移動と数える）。

**Why**: surprise / look_at / petted が並列で粒度がばらばらのまま twitch / flick / sag / nuzzle / stretch を
足すと破綻する（レビュー Part 3）。展示会場で「もう少し臆病に」を config 1 行で試せるかが当日の完成度を決める。
一次遅れの Stress は接近「距離」を積分するだけで歩行と突進を区別できないので、速さのしきい値で分けた。

**Alternatives**: (1) Expression に関数を足し続ける (2) 状態機械の中にしぐさを書く
(3) 突進を approach_ref を上げて吸収する（歩行との区別が付かない）。

**Trade-offs**: 文法の表は config が長くなる（未知の語彙は起動時に弾く）。grammar は brain の状態を
毎周期見るので、状態遷移が _on_enter を通らない起動直後は tick 側で補う。数値はすべて模擬で、人の評価が未。

---

## 2026-09-18 — 実画像経路は観測器で差し込み、録画は実機の開始条件を満たさない。見失い後は人が確認する

**Decision**: `CameraObserver`（カメラ番号 / 動画 / 静止画）を `SimSession(observer=)` で差し込む。
`pose_source` はカメラ番号のときだけ `aruco`、動画・静止画は `aruco_file`（実機の開始条件を満たさない）。
録画はループ再生しない。自己位置は最初と見失った後、人が K で確認するまで走行に使わない。

**Why**: 旧 `RealCamera` は人の位置しかセッションへ渡さず、自己位置は入っていなかった。
録画を再生しながら実機を走らせると機体の実位置と無関係な位置で動くので、出どころで区別する。
偽マーカで再取得した位置のまま自動再開しないために、人の確認を段に入れた（Phase 4 の残課題）。

**Alternatives**: (1) 録画も `aruco` 扱い（試験は楽だが事故の経路になる）
(2) 自動再開（展示の手間は減るが、誤った位置で走る）。

**Trade-offs**: 展示中、来場者がマーカを隠すたびに K → G の操作が要る。
実カメラの画像ではまだ試していない（録画は仮想カメラ製。照明・歪みは含まない）。

---

## 2026-09-16 — Vision は「観測器」として差し込み、見失ったら保持して自動再開しない

**Decision**: `SimSession(observer=...)` で観測の経路を差し替える。既定は真値（`GroundTruthObserver`）、
Phase 4 は `SimVisionObserver`（仮想カメラ画像に本物の ArUco 検出）。`pose_source` は `aruco_sim` とし、
実機の開始条件（`aruco`）を満たさない。自己位置が `pose_max_age_s` より古くなったら通常停止（HOLD, source 知覚）し、
**位置が戻っても自動で再開しない**。Vision を通す模擬は待機から始まる。
同じ ID の重複マーカは使わず、前回位置から `max_speed_mm_s × 経過 + jump_margin_mm` を超える観測は捨てる。
位置不明から戻るには首・尾の両方が要る。

**Why**: 故障注入で、(1) 重複 ID の採否が検出順の偶然 (2) 首が隠れると偽マーカで位置が飛ぶ
(3) 行動が自己位置の古さを見ていない、の3つが見つかった。展示では来場者の手がマーカを隠すのは日常なので、
見失い → 保持は必須。自動再開は「位置が戻った」判定が偽マーカで騙されうる（再取得で 839mm ずれた）ため採らない。

**Alternatives**: (1) 真値に雑音を足して Vision の代わりにする（検出・視差・遮蔽の経路を試せない）
(2) 位置が戻ったら自動再開（展示の手間は減るが、偽の再取得で走る）
(3) マーカ間距離の一貫性で偽マーカを弾く（曲がると弦長が変わり、近い偽物は区別できない）。

**Trade-offs**: 1 フレーム 約 45ms の描画 + 検出で、試験が 20 秒ほど増えた。
来場者がマーカを隠すたびに操作者の再開操作が要る。近い偽マーカは依然として区別できない（運用規則で避ける）。

---

## 2026-09-16 — ELECTRICAL_SAFETY_GATE を自律走行の開始条件に入れ、接触→脱力は要求整理で止めた

**Decision**: `safety_limits.electrical_safety_gate` の8項目（実電流・電源容量・過電流保護・配線発熱・
独立電源遮断・物理 E-STOP・サーボ温度・実停止時間）が**証拠つき COMPLETE** になるまで、
`autonomy_blockers()` が実機の自律走行を拒否する。手動の計測通電は止めない（測れないとゲートを埋められない）。
接触→脱力 20ms は実装せず、`docs/contact_release_requirements.md` に要求の穴（Q1〜Q6）と時間予算を書いた。

**Why**: 電気的制限（層2）が丸ごと無いことは文書にあったが、コードの上では何も止めていなかった。
「未完了」を文書でなく開始条件として持つと、ソフトがそろった日に黙って走り出すことが無くなる。
接触→脱力は「何を・何で・どう反応するか」が未定で、しきい値は実測が無いと置けない。
既定値つきのスイッチだけ作ると実装済みに見えるので作らなかった。
計算の途中で、1:191 の高減速比では**脱力しても挟み込みが解けない可能性**に気づいた（バックドライブ未測定）。

**Alternatives**: (1) RealSnake の接続自体を拒否する（初回の計測までできなくなる）
(2) 仮想機体に負荷しきい値の脱力を先に実装する（値の根拠が無い）。

**Trade-offs**: 実機の自律走行は当面どうやっても開始できない（意図どおり）。
試験用に config を書き換える場合も `evidence` が要る。

---

## 2026-09-16 — ARM に boot_id を持たせ、再起動後の自動再走行を機体側で拒否する

**Decision**: `ARM` の payload に「どの起動の機体を ARM するのか」を示す `boot_id`(u16) を持たせ、
機体は自分の `boot_id` と違う ARM を **NACK(`STALE_BOOT`)** で拒否する。

**Why**: 「機体が再起動したら PC は自動で ARM しない」を **PC 側の判断だけ**で実装していたため、
再起動の telemetry が届く前に出た ARM が通っていた。閉ループ試験で
**人の操作なしに 240ms で走行が再開する**ことを実際に観測した（`test_reboot_during_following...`）。
送り手の遠慮では守れない。受け手（機体）が拒否して初めて保証になる。

**Alternatives**:
(1) PC 側で「telemetry を受けるまで ARM しない」→ すれ違いは残る（送信は非同期）。
(2) 再起動後しばらく ARM を受け付けない（時間で守る）→ 時間は環境で変わる。根拠が弱い。

**Trade-offs**: `ARM` の payload が 0 → 2 バイトに増え、テレメトリを一度も受けていない PC は
ARM できなくなった（どの機体か分からないので、正しい制約）。

**Context**: 2026-09-16。Stage M（閉ループ）で発見。ファームにも同じ検査を入れ、
`tests/test_phase2_invariants.py` に回帰試験を 2 件追加。

## 2026-09-16 — 可動域を「意味の違う3つ」に分け、±50°（CONDITIONAL）を clamp にした

**Decision**: CAD の最新情報（64.8° 干渉なし / 64.9° 干渉）を受けて、可動域を
`geometry_collision_onset`（64.8〜64.9 の bracket）/ `mechanical_design_limit`（±55 PROVISIONAL）/
`software_operational_limit`（±50 CONDITIONAL）に分け、**clamp に使うのは最後のものだけ**にした。
`verified_in_cad` / `verified_with_cable` / `verified_with_hardware` を保持する。

**Why**: 「±64° = 安全限界」と扱っていたのは誤りだった。64.8/64.9 は **nominal geometry で
干渉が始まる境界**であって、運用してよい角度ではない。Cable Routing も不合格のまま。
名前を分けないと、また同じ取り違えが起きる。

**Trade-offs**: 旧とぐろ（J1=83°）が入らなくなった。**clamp して同じ名前で使わない**と決め、
`legacy_poses`（SIMULATION_LEGACY_ONLY）へ移して、R03 用の緩い弧（`rest_arc`）を作った。
展示用のとぐろは R03 の範囲で別途再設計する。

**Context**: `docs/safety_limits.md` / `tests/test_safety_limits.py`。±50° は
**物理試験へ進むための候補**であって、実機で許可された値ではない。

## 2026-09-16 — サーボは STS3215-C044（7.4V/1:191）。トルクは4つに分ける

**Decision**: `servo` を C044 プロファイルへ置き換え、旧 12V・1:345（30kgf·cm）の値を削除した。
トルクは `rated_torque_reference`(0.510N·m) / `stall_torque_reference`(1.569N·m) /
`software_torque_limit`(0.450N·m) / `measured_safe_torque`(**null = UNKNOWN**) に分ける。

**Why**: ストールトルクを連続安全トルクとして使うと、実機で確実に焼く。
また REFERENCE 値（CAD 資料）と実測値を同じ場所に置くと、いつのまにか「確定値」として扱われる。

**Trade-offs**: 上限が 1.18N·m → 0.450N·m に下がり、MuJoCo では**トルク飽和が 12%** 出た。
これは「歩容がサーボの定格に対して重い」という設計上の信号なので、隠さず残す。

**Context**: 実測前に安全トルクを確定しない。`hardware_verified: false`。

## 2026-09-15 — 機体の状態を7つにし、機体の中に仮想サーボバスを置いた

**Decision**: Virtual ESP32 の状態を `BOOT / DISARMED / ARMED_HOLD / DRIVING / FAULT_HOLD /
EMERGENCY_LATCHED / TORQUE_DISABLED` の7つにした。あわせて、機体の中に `MockServoBus` を置き
（`VirtualServoBus`）、**テレメトリが返すのは指令角ではなく模擬の実測角**にした。
応答しない軸は欠けたまま返し、走行中に欠けたら `FAULT_HOLD` へ落とす。

**Why**: 3状態（DISARMED / ARMED / EMERGENCY）では「異常で止まっている」と「人が止めた」が
区別できず、停止理由を別フィールドで補っていた。実機ファームも同じ区別が要る。
また、指令角をそのままテレメトリにすると「指令したのに追従していない」が**原理的に見えない**。
引っかかり・過負荷・配線抜けはそこにしか現れない。

**Alternatives**: (1) 3状態のまま stop_reason で読み替える → 状態と理由の対応を毎回推論することになる。
(2) 仮想サーボを入れず、テレメトリに指令角を返し続ける → 実機で最初に効く異常が模擬できない。

**Trade-offs**: 状態の値が変わったので、既存テスト 18 件の期待値を書き換えた。
模擬サーボを毎周期読むぶん少し遅くなる（計測では suite 全体で有意差なし）。

**Context**: 実機ゼロ。`tests/test_firmware_sync.py` を足して、ファームと Python の値のズレを
自動検出するようにした（実際に MAX_PAYLOAD と可動域 ±85° の残りを検出した）。

## 2026-09-15 — 摩擦の値は belly プロファイルだけが持つ

**Decision**: `sim.tangential_drag_ratio` / `sim.pad_drag_ratio` を削除し、
`belly.profiles[type][profile]` を唯一の出どころにした。`tools/fit_sim.py` もそこを同定する。

**Why**: Belly（wheel / snake）を切り替えられるようにした結果、同じ意味の値が config の2か所に
現れた。二重管理は必ずズレる（実際、テストが `sim` 側を書き換えても効かなくなっていた）。

**Trade-offs**: 既存の設定ファイルと手順書の記述を書き換える必要があった。

## 2026-09-15 — 3D 物理は MuJoCo を第一候補にする（まだ導入しない）

**Decision**: Phase 3B の物理エンジンは MuJoCo を第一候補とし、**`serpens/` 本体からは import しない**
任意の依存（`tools/` からのみ）として入れる。モデルは config から生成する。

**Why**: Snake Belly の是非は**異方性摩擦**でしか判定できない。MuJoCo は接触ごとに
5つの摩擦係数（接線2・ねじり1・転がり2）を持ち、接線方向に別々の値を入れられる。
Apache-2.0、`pip install mujoco` で本体同梱、CPU で動く。

**Alternatives**: PyBullet（等方摩擦が基本）/ Gazebo（ROS 寄り・別インストール）/
Isaac Sim（GPU 必須・重い）/ Drake（大きい）。

**Trade-offs**: 依存が1つ増える。2D シミュレータは残す（行動の回帰試験は速度が要る）。

**Context**: 一次資料で確認（`docs/phase3b_physics_sim.md` の出典）。**まだ何も実装していない。**

## 2026-09-14 — 出力先を RobotInterface で差し替え可能にした（Phase 3）

**Decision**: セッションは毎周期 `MotionCommand`（9軸の合成角 + 歩容パラメータ + 胴体ベース角 +
首・頭の角 + 呼吸）を出力先へ渡す。出力先は `DirectRobot`（サーボへ角度を直接書く）と
`LinkRobot`（ESP32 へ DRIVE / BODY / HEAD を送る）の2つ。`--robot link` で切り替える。
リンク経路では**ローカルのサーボバスを作らない**（`session.bus is None`）。

**Why**: 行動が決めているのは元から「歩容パラメータ」（`Brain._set_drive` が `GaitParams + γ0` を作る）
なのに、出力の直前で 9軸の角度へ潰していた。そのため駆動リンク（機体が歩容を作る前提）へ流せず、
**アプリ経路では「PC が死んでも機体が止まる」が成立していなかった**。角度とパラメータの両方を
持つ指令にすることで、どちらの出力先でも同じ行動コードが使える。

**Alternatives**:
(1) `ServoBus` の実装としてリンクを作る（`LinkServoBus`）。→ 却下。バスが受け取るのは角度なので、
    50Hz で角度列を送ることになり「機体が歩容を作る」性質が消える。PC が死んだ瞬間の安全性も戻らない。
(2) セッションを2本（シミュレーション用・リンク用）に分ける。→ 却下。行動と安全の要が二重化する。

**Trade-offs**: `MotionCommand` は角度とパラメータを両方運ぶので情報が重複している。
世界の更新と画面表示に角度が要るため受け入れた。`hold()` はリンク経路では何もしない
（保持は機体の仕事）ので、経路によって意味が違うメソッドが1つある。

**Context**: 実 ESP32 は無い。模擬 ESP32（`SimulatedDevice`）を相手に
`tests/test_phase3_link_robot.py`（9件）で、走行・PC 強制終了・USB 抜去・通常停止・緊急停止のラッチ・
機体の再起動・脱力を確認した。

## 2026-09-14 — 機体の ARM は「人の開始操作」でしか行わない

**Decision**: `LinkRobot` が ARM を送るのは (a) 走行中に機体が待機だったとき、(b) 人が開始操作を
したとき（`on_operator_start`）。ただし**機体が再起動した記録（`client.rebooted`）が立っている間は
周期処理から ARM しない**。解除できるのは人の開始操作だけ。緊急停止の解除（`CLEAR_FAULT`）も
人の操作からのみ送る。

**Why**: Phase 2 完了条件 6「ESP32 再起動で自動再開しない」を、PC 側が壊せてしまうため。
リンクが繋がっていれば PC は毎周期 ARM を送れる立場にあり、放置すると「機体が再起動 → PC が即 ARM →
走り出す」になる。

**Trade-offs**: 展示中に機体が再起動すると、人が G を押すまで動かない。**それが正しい**と判断した。

**Context**: `tests/test_phase3_link_robot.py::test_device_reboot_does_not_auto_resume`。

## 2026-09-14 — トルク比を「安全上限に対する割合」に変えた

**Decision**: `ServoBus.set_torque_limit(id, ratio)` の `ratio` を
**ストールトルクに対する割合ではなく、安全上限（`safety_limits.torque_ratio_max` = 0.40）に対する割合**
に変えた。`ratio=1.0` でもストールの 40%（= 1.18 N·m ≤ 構想設計書 16章の 1.2 N·m）までしか出ない。
接続のたびにセッションが `apply_torque_ceiling()` を通し、書けなかった軸があると
`autonomy_blockers` が実機の自律走行を止める。

**Why**: 上限を入れる前は、脱力演出の終わりに `set_torque_limit(sid, 1.0)` が呼ばれていた。
上限をレジスタへ書くだけの設計にすると、**演出が終わるたびに全力（2.94 N·m）へ戻る**経路が残る。
安全の上限を「誰も通り抜けられない場所」に置くには、公開 API 自体を上限相対にするしかない。

**Alternatives**:
(1) 演出側で `ratio × ceiling` を計算する。→ 却下。呼び出し側が1つ増えるたびに同じ掛け算を
    忘れる余地が増える。安全は呼び出し側の規律に依存させない。
(2) 接続時に上限を書くだけにする。→ 却下。上記のとおり演出の経路で上書きされる。
(3) `connect()` の中で上限を書く。→ 却下。応答しない軸があると接続そのものが例外になる。
    接続は成立させ、上限を書けなかった事実を**開始条件の判定**へ回す方が扱いやすい。

**Trade-offs**: 脱力演出の実効トルクが 0.6 → 0.24（ストール比）に下がる。実機で
「撫でたときに首が落ちない」かは未検証。落ちるなら `poses.relax.torque_ratio` を上げて調整する
（上限そのものは動かさない）。テスト 3 件がこの意味変更に追従した。

**Context**: 実サーボはまだ 1 個も無い。レジスタ 48 の単位は資料間で食い違いがあり
（`docs/sts3215_registers.md` §注意2）、**実機で要検証**。

## 2026-09-14 — 通信断で「とぐろ」へは移らず、その場で保持する

**Decision**: 構想設計書 16章は通信障害時に `coil_and_hold`（とぐろ + 脱力 + 待機）を挙げているが、
本機は**その場で姿勢を保持**する（`docs/link_protocol.md` の HOLD）。とぐろへは移らない。

**Why**: 通信が切れた直後は PC が状況を見られない。その状態で胴体 6軸を大きく動かすのは、
人や物の位置が分からないまま大きな動作をすることになる。保持なら新しい危険を作らない。
到達時間も 400ms（構想設計書の目標 3 秒より速い）。

**Alternatives**: (1) 仕様どおりとぐろへ移る (2) ゆっくりホーム姿勢へ戻す。
→ どちらも「止まる」より多く動く。停止で動くのが最も危険。

**Trade-offs**: 長時間の通信断で、鎌首など重心の高い姿勢のまま保持し続けることがある。
熱と電力の観点では不利。とぐろは充電ドッキングの文脈で、人の操作か機体の自己判断が
確立してから足す。

**Context**: Phase 2 の機体側 watchdog は実装済みだが、実機のファームはまだ書き込んでいない。

## 2026-09-14 — 制御ループ例外時、fault の公開を緊急停止の後に移した

**Decision**: `ControlLoop._on_fault()` で `self.fault` への代入を、
`session.request_emergency()` と `enforce_stop_output()` が終わった**後**へ移した
（`serpens/runner.py`）。

**Why**: `self.fault` は GUI と監視が「異常を検知した」として読む唯一の合図。
これを先に代入すると、fault が外から見えているのに機体はまだ止まっていない瞬間が生まれる。
実際にその隙間が観測できていて、`test_control_loop_exception_triggers_emergency_and_is_reported`
が「fault は立っているのに `stop.latched` が False」で落ちていた。
安全機構の実装はどこも正しく（`StopSupervisor.emergency()` は無条件にラッチする）、
公開の順序だけが間違っていた。

**Alternatives**:
(1) テスト側を直す（fault を見た後に latched を待つポーリングを足す）。
    → 却下。テストが観測していたのは実在の危険な隙間で、テストの方が正しい。
    Home AI 構想設計書の第一原則「安全は最下層で保証する」に反する状態を隠すことになる。
(2) `_on_fault` 全体をロックで囲む。
    → 過剰。単一代入の順序の問題で、公開を最後に回せば十分。

**Trade-offs**: 停止要求が例外を投げた場合、その情報が `fault` に載るのが数ミリ秒遅れる。
ログ (`log.exception`) は従来どおり即時に出るので、観測性は落ちていない。

**Context**: 全体テストは修正前 204 passed / 1 failed、修正後 205 passed。
この失敗は間欠的（タイミング依存）で、環境が速いと通ることがあった。

---

## 2026-09-14 — フォルダを C:/2026/serpens へ移動しないことにした

**Decision**: canonical 名は `serpens` だが、実際の場所は `Serpens_Home AI/serpens` のまま置く。
対応は `C:/2026/projects.json` と `C:/2026/PORTFOLIO.md` が持つ。

**Why**: この repo は `.venv` を同梱していて、`pyvenv.cfg` と `Scripts/` 内の実行シムは
**絶対パスを埋め込んでいる**。移動すると venv が壊れ、復旧には
`requirements.txt` + `requirements-nodeps.txt` の 2 段階インストール（opencv の競合を避ける手順）を
ネットワーク付きでやり直す必要がある。
「フォルダ名を揃える」ことの価値より、動いている環境を壊すリスクの方が大きい。
ユーザーの指示でも「プロダクトが壊れないようにすること」が優先と明示されている。

**Alternatives**: (1) 移動して venv を作り直す (2) シンボリックリンクを張る
（Windows では管理者権限か開発者モードが要り、git と相性が悪い）。

**Trade-offs**: 設計上の名前と実際のパスが一致しない。台帳を見ないと分からない。
移動したくなったら、venv を作り直せる時間があるときに `projects.json` の `path` を
更新して同期を回せば済むようにしてある。

---

## 2026-09-14 — Autonomous Product Development OS を導入した

**Decision**: 共通の作業 OS を `CLAUDE.md` の生成ブロックとして持ち、
プロジェクト固有の状態を `agent/STATE.md` / `ROADMAP.md` / `DECISIONS.md` に分けた。

**Why**: 複数セッションにまたがる作業で、毎回リポジトリを読み直すところから
始まるのを避けるため。共通ルール（OS）と、プロジェクト固有の記憶を分離すると、
OS を 1 箇所で更新でき、記憶はプロジェクトに残る。

**Alternatives**: (1) 何もしない (2) 全部 README に書く
(3) ルートの共通ファイルを `@import` する。

**Trade-offs**: OS 本文が各リポジトリに複製されるので、更新には
`python tools/sync_agent_os.py`（C:/2026 側）の実行が要る。
その代わり、このリポジトリ単体を clone しても OS が欠けない。

**Context**: C:/2026 配下の各プロダクトは独立した git リポジトリ（または未管理）で、
共通の親リポジトリが無い。
