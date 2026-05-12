# 環状道路 渋滞シミュレーション (IDM)

## 概要

環状道路（リング状の1本道）に複数台の車を並べ、**IDM (Intelligent Driver Model)** で各車の挙動を計算する MAS シミュレーションです。最初に1台だけ停止させる「摂動」を加えると、**stop-and-go wave（渋滞波）** が自然に発生し、リングを逆向きに伝搬していく様子が観察できます。

実車も信号もないのに渋滞が生じる、有名な現象の再現です（実車実験では Sugiyama et al. 2008 が有名）。

---

## ファイル構成

シミュレーション本体と可視化を**2ファイルに分離**しています。理由は、実験でパラメータを変えながら何度も走らせるとき、毎回 matplotlib のアニメーション処理を通すのは非効率（処理時間の大半が描画になる）だからです。

| ファイル | 役割 | 依存 |
|---|---|---|
| `traffic_sim.py` | シミュレーション本体 | numpy のみ |
| `visualize_animation.py` | アニメーション + GIF 保存 | `traffic_sim` + matplotlib |

`traffic_sim.py` を import すれば、可視化を介さずに大量のパラメータスイープが回せます。アニメーションを見たいときだけ `visualize_animation.py` を実行します。

---

## IDM（Intelligent Driver Model）

各車の加速度を以下の式で計算します（Treiber et al. 2000）。

$$
\frac{dv}{dt} = a \left[ 1 - \left( \frac{v}{v_0} \right)^\delta - \left( \frac{s^*}{s} \right)^2 \right]
$$

$$
s^* = s_0 + \max\!\left( 0,\ v \cdot T + \frac{v \cdot \Delta v}{2\sqrt{a \cdot b}} \right)
$$

- 第2項 `(v/v0)^δ` … 希望速度に近づくほど加速をやめる「**フリー走行項**」
- 第3項 `(s*/s)^2` … 前車との距離が `s*` を切ると急ブレーキする「**相互作用項**」
- `s*` … 速度依存の目標車間距離（速いほど大きい）

つまり「自由なら加速、詰まったら減速」という直感的な振る舞いを、1本の式で表現したモデルです。

---

## パラメータ一覧

パラメータは2つのデータクラスで管理します。

### `IDMParams`（IDM モデル本体のパラメータ）

| 記号 | フィールド名 | デフォルト | 意味 |
|---|---|---|---|
| v₀ | `v0` | 16.0 m/s | 希望速度（約58 km/h）|
| T | `T` | 1.2 s | 安全車間時間 |
| a | `a_max` | 0.8 m/s² | 最大加速度 |
| b | `b_comf` | 1.5 m/s² | 快適減速度 |
| δ | `delta` | 4 | 加速度指数 |
| s₀ | `s0` | 2.0 m | 最小車間距離 |
| l | `l_veh` | 5.0 m | 車長 |

### `SimParams`（シミュレーション設定）

| フィールド名 | デフォルト | 意味 |
|---|---|---|
| `ring_length` | 350.0 m | 環状道路の周長 |
| `n_cars` | 28 台 | 車の台数 |
| `dt` | 0.25 s | 時間刻み |
| `n_steps` | 600 | 総ステップ数 |
| `perturbation` | `'stop_one'` | 摂動の種類（`'stop_one'` / `'small'` / `'none'`）|

ポイントは **車1台あたりの平均占有距離 = 350/28 ≈ 12.5 m** で、IDM の安定領域の境界付近に車間を設定していること。これにより少しの摂動で渋滞波が立ち上がります。

データクラスを使う利点は、実験のたびに `IDMParams(T=0.8)` のように一部だけ書き換えてポンと渡せること。元コードのようにグローバル定数を上書きする必要がなくなります。

---

## `traffic_sim.py` の中身

### 状態の持ち方

全車の位置と速度を **2本の numpy 配列**（`x`, `v`、長さ N）で持ちます。元コードは `Car` クラスをリストにしていましたが、numpy 配列にすると全車一括のベクトル演算が使えて10〜100倍速くなります。

### `initialize_state(idm, sim)`

- N 台を環状道路上に **等間隔** に配置
- 全車を希望速度の 60%（= 9.6 m/s）で発進
- `perturbation` 設定に応じて摂動を入れる：
  - `'stop_one'`：1台だけ速度0（デフォルト、強い摂動）
  - `'small'`：1台だけ 5% 落とす（弱い摂動）
  - `'none'`：完全均一（浮動小数点誤差だけが摂動）

### `step(x, v, idm, sim)`

1ステップ分の更新を **全車一括の numpy 演算** で行います。流れは：

1. `np.argsort(x)` で位置順にソート（前車を特定するため）
2. `np.roll` で各車の前車を取得
3. 車間を mod で計算（リング一周を処理）
4. IDM の加速度を全車一括計算
5. 速度・位置を更新

元コードの for ループ版と数学的には等価ですが、Python のオーバーヘッドが消えて高速。

**注意点**：加速度計算と状態更新を分けているのは、同一ステップ内で計算順序による依存を出さないため（一斉に同じ時刻の状態を参照したい）。numpy 演算で自然にこれが達成されています。

### `run_simulation(idm, sim, record_positions=False)`

メインループ。`record_positions=False` のときは各ステップの平均速度・最低速度だけを記録（メモリ節約）。`True` にすると全車の全ステップの位置・速度も記録します（時空間プロット用）。

戻り値は dict：

```python
{
    'avg_v': np.array,  # (n_steps,) 各ステップの平均速度
    'min_v': np.array,  # (n_steps,) 各ステップの最低速度
    # record_positions=True のとき以下も
    'x_history': np.array,  # (n_steps, n_cars)
    'v_history': np.array,  # (n_steps, n_cars)
}
```

### `summary_stats(result, transient=200)`

過渡状態（最初の transient ステップ）を除いた定常部分の要約統計を返します。実験でパラメータの影響を比較するときに使う指標：

- `mean_avg_v`：平均速度の時間平均（流れの良さ）
- `mean_min_v`：最低速度の時間平均（渋滞の深刻さ）
- `std_avg_v`：平均速度のばらつき（波が立っているかの指標）
- `wave_amplitude`：波の振幅（平均と最低の差）

---

## `visualize_animation.py` の中身

`traffic_sim.py` を import して、`record_positions=True` でシミュレーションを走らせ、その履歴をアニメーション化します。

- **左図**：環状道路上の車を散布図で表示。色は速度（緑=速い、赤=遅い）
- **右図**：時間に対する平均速度・最低速度の推移

`to_xy()` で弧長を `(cos θ, sin θ)` に変換し、円周上に車をプロット。GIF 保存機能もそのまま。

このファイルは元コードの可視化部分を **そのまま移植** したもので、振る舞いも見た目もほぼ同等です。違いは内部のシミュレーション計算が高速化されている点だけ。

---

## 実験で使うときの典型パターン

`traffic_sim` を import するだけで、いろんな実験が数行で書けます。

### 例1：デフォルト設定で1回走らせて統計を見る

```python
from traffic_sim import run_simulation, summary_stats

result = run_simulation()
print(summary_stats(result))
```

### 例2：パラメータを変えて走らせる

```python
from traffic_sim import IDMParams, SimParams, run_simulation, summary_stats

# 車間時間を長めにしたケース
idm = IDMParams(T=1.8)
sim = SimParams(n_cars=28)
result = run_simulation(idm, sim)
print(summary_stats(result))
```

### 例3：台数スイープ

```python
import matplotlib.pyplot as plt
from traffic_sim import SimParams, run_simulation, summary_stats

n_values = range(10, 41)
mean_v = []
for n in n_values:
    r = run_simulation(sim=SimParams(n_cars=n))
    mean_v.append(summary_stats(r)['mean_avg_v'])

plt.plot(list(n_values), mean_v, 'o-')
plt.xlabel('N')
plt.ylabel('Mean velocity')
plt.show()
```

### 例4：時空間プロット用に履歴を取る

```python
result = run_simulation(record_positions=True)
x_hist = result['x_history']  # (n_steps, n_cars)
v_hist = result['v_history']
# あとは pcolormesh で時空間図を描く
```

---

## 観察ポイント

`visualize_animation.py` を実行すると次のような現象が見られます。

1. **0〜数十ステップ**：停止した1台に後続車が詰まり、局所的な渋滞が発生
2. **中盤**：渋滞の塊が **車の進行方向と逆向きに** ゆっくり伝搬する（stop-go wave）
3. **後半**：摂動が消えず、定常的な波として残り続ける
4. **右の速度グラフ**：平均速度は中程度で振動、最低速度は周期的に0近くまで落ちる

ドライバーは合理的に振る舞っているのに、系全体としては渋滞が消えない、という創発現象です。

---

## カスタマイズの目安

| やりたいこと | 操作 |
|---|---|
| 渋滞を消したい | `n_cars` を減らす（例: 20）か `T` を長く（例: 1.8） |
| 渋滞をひどくしたい | `n_cars` を増やす（例: 35）か `b_comf` を小さく（例: 1.0） |
| 摂動を弱める | `perturbation='small'`（5% 減速）か `'none'`（誤差のみ） |
| 速い走行に揃える | `v0` を上げる（例: 22 = 約80 km/h） |
| 2車線にしたい | 状態配列を2次元化し、車線変更ルール（MOBIL など）を実装 |
| 信号や合流を追加 | リング上の特定位置に制約を加える |

---

## 性能の目安

- 1 sim（28台 × 600ステップ）：約 50 ms
- パラメータスイープ 100 ケース：約 5 秒（シリアル実行）
- アニメーション + GIF 保存：30 秒〜数分（描画が支配的）

つまり実験で何百ケース走らせてもシミュレーション本体は数秒〜十数秒で終わります。**遅さの正体は基本的にアニメーション処理**なので、実験中はそこを通さないのが鉄則です。