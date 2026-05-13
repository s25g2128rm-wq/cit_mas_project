# -*- coding: utf-8 -*-
"""
実験2：N スイープ（臨界密度の発見）

目的::
車の台数 N を 10〜40 で動かし、定常状態の平均速度と最低速度をプロットする。
ある N* を境に最低速度がゼロに張り付き始めるはず。これが臨界点。

使い方:
    python experiment2_N_sweep.py

出力:
    - exp2_N_sweep.png  : 結果のグラフ
    - exp2_data.npz     : 生データ（後でプロット調整したいとき用）
"""

import numpy as np
import matplotlib.pyplot as plt
from traffic_jam_mas import IDMParams, SimParams, run_simulation, summary_stats


def main():
    # ===== スイープする N の値 =====
    n_values = list(range(10, 41))  # 10, 11, ..., 40 (31ケース)

    mean_v_list = []
    min_v_list = []
    std_v_list = []

    print(f'Running {len(n_values)} simulations...')
    for i, n in enumerate(n_values):
        sim = SimParams(n_cars=n, n_steps=800)  # 過渡を抜けるよう余裕を持たせる
        result = run_simulation(sim=sim)
        stats = summary_stats(result, transient=300)
        mean_v_list.append(stats['mean_avg_v'])
        min_v_list.append(stats['mean_min_v'])
        std_v_list.append(stats['std_avg_v'])
        print(f'  [{i+1:2d}/{len(n_values)}] N={n:2d}  '
              f'mean_v={stats["mean_avg_v"]:5.2f}  '
              f'min_v={stats["mean_min_v"]:5.2f}')

    # ===== 結果を保存（後でプロット調整したいとき用）=====
    np.savez('exp2_data.npz',
             n=n_values, mean_v=mean_v_list,
             min_v=min_v_list, std_v=std_v_list)
    print('Data saved to exp2_data.npz')

    # ===== プロット =====
    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(n_values, mean_v_list, 'o-', label='Mean velocity',
            color='C0', markersize=6)
    ax.plot(n_values, min_v_list, 's-', label='Min velocity',
            color='C3', markersize=6)
    ax.set_xlabel('Number of cars N')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Experiment 2: Velocity vs Number of cars (ring length = 350m)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # 上軸に密度を併記
    ax2 = ax.twiny()
    ax2.set_xlim(ax.get_xlim())
    ticks = ax.get_xticks()
    ax2.set_xticks(ticks)
    ax2.set_xticklabels([f'{t/350:.3f}' for t in ticks])
    ax2.set_xlabel('Density (cars / m)')

    plt.tight_layout()
    plt.savefig('exp2_N_sweep.png', dpi=120)
    print('Plot saved to exp2_N_sweep.png')
    plt.show()


if __name__ == '__main__':
    main()