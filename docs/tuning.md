# Tuning reference

Open **Right Shift → 调校**. Controls are grouped by purpose. **恢复默认** restores the current category; on the category index it restores all tuning. The `parameters` object in `config/turneler.json` uses the same registry as the UI. The route horizon remains `icerHorizon` (default 48, range 24–72 blocks).

Controller defaults follow the Icer specification. Existing values of retained keys survive; restore tuning defaults to adopt the new coefficients. Removed `narrow_brake`, `narrow_start`, `narrow_band`, `yaw_deadzone`, `launch_speed`, `launch_angle` and `launch_thrust` are ignored and removed on save. A narrow straight still requests full forward thrust; no setting introduces a maximum or target speed. Bend braking subtracts natural ice drag, and neutral steering ticks retain yaw inertia.

Feedback half-width bounds keep broad ice from delaying drift correction. S releases once per five ticks, or ten ticks at speeds up to 0.55 blocks/tick. Its low-speed period is normalized to be at least the high-speed period. These thresholds choose reverse pulse timing, not a speed cap. S releases and painted-route coast braking keep A/D feedback and the physical 0.005 bare-turn thrust. Controls referring to painted geometry apply when a route supplies `paintedHalfWidth`; the current local terrain generator uses measured widths.

Route averaging retains its default 20-tick window. Points with a full window of actual tick contributions become fixed; subsequent refreshes preserve the stable prefix and replace the tail. `history_ticks` set to 1 disables averaging and freezing. Changing tuning or horizon discards pending work and re-anchors from measured state. Restoring defaults preserves settings navigation and scroll history.

Ranges bound heuristics; they do not guarantee every combination follows every track. Non-finite values use defaults, integer controls are rounded, and related bounds are normalized. Vanilla thrust, hull dimensions, friction inputs, numerical tolerances and conservative full-hull sampling remain fixed.

## 路线生成

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 扫描步长 | `coarse_step` | 3.0 格 | 0.75–6.0 格 | 0.25 | 常规路线扫描的前进距离。减小可提高精度。 |
| 精细扫描步长 | `fine_step` | 1.5 格 | 0.25–3.0 格 | 0.25 | 常规扫描被截断时，用更短的步长重试。 |
| 候选路线数 | `beam` | 12.0 | 4.0–32.0 | 1.0 | 每轮保留的路线数。越大越容易发现弯道，也越耗时。 |
| 扫描转角 | `turn_range` | 35.0° | 10.0–65.0° | 1.0 | 每一步向左右探索的最大角度。 |
| 两侧探测范围 | `width_limit` | 8.0 格 | 2.0–16.0 格 | 0.5 | 测量冰面宽度时，向每侧探测的最大距离。 |
| 起点搜索范围 | `start_search` | 3.0 格 | 0.5–8.0 格 | 0.25 | 在船身两侧寻找可通行起点的范围。 |
| 中心搜索范围 | `section_search` | 2.5 格 | 0.5–6.0 格 | 0.25 | 每个扫描点向两侧寻找安全中心的范围。 |
| 中心修正上限 | `centre_shift` | 2.5 格 | 0.25–4.0 格 | 0.25 | 单个扫描点允许向冰面中心移动的距离。 |
| 旧路线引导距离 | `guide_distance` | 5.0 格 | 1.0–12.0 格 | 0.5 | 沿上一条路线向前取方向，减少新路线抖动。 |
| 旧路线有效距离 | `incumbent_distance` | 4.0 格 | 1.0–8.0 格 | 0.5 | 船与旧路线的距离超过此值时，重新探索方向。 |
| 速度方向启用值 | `travel_threshold` | 0.18 格/tick | 0.05–0.8 格/tick | 0.01 | 低于此速度时，扫描方向优先使用船头方向。 |

## 路线评分

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 宽度偏好 | `margin_weight` | 0.55 | 0.0–2.0 | 0.05 | 更偏好两侧留有余量的路线。 |
| 转弯代价 | `turn_weight` | 0.32 | 0.0–2.0 | 0.01 | 增大后更偏好平缓的路线。 |
| 横向修正代价 | `shift_weight` | 0.1 | 0.0–1.0 | 0.01 | 降低单步中心位置大幅改变的倾向。 |
| 路线连续性 | `incumbent_weight` | 0.07 | 0.0–1.0 | 0.01 | 增大后更偏好与上一条路线接近的候选。 |

## 曲线拟合

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 平滑次数 | `smooth_passes` | 8.0 | 0.0–16.0 | 1.0 | 滤除方块边缘带来的中心线抖动。窄弯会自动减少平滑。 |
| 锚点间隔 | `anchor_stride` | 4.0 | 1.0–8.0 | 1.0 | 每隔多少个扫描点拟合一个曲线锚点。 |
| 平滑余量比例 | `smooth_room` | 0.45 | 0.1–1.0 | 0.05 | 曲线允许偏离扫描中心的距离与冰面余量之比。 |
| 最小拟合偏移 | `smooth_min` | 0.25 格 | 0.05–0.8 格 | 0.05 | 曲线拟合允许的最小中心偏差。完整船体检查仍然生效。 |
| 最大拟合偏移 | `smooth_max` | 1.2 格 | 0.1–2.5 格 | 0.1 | 曲线偏离扫描中心的最大距离。 |
| 曲线控制柄长度 | `handle_scale` | 1.0 | 0.25–1.5 | 0.05 | 调节曲线圆滑程度。失败时自动尝试更短控制柄。 |
| 路线平均窗口 | `history_ticks` | 20.0 tick | 1.0–40.0 tick | 1.0 | 按真实 tick 对齐原始路线并取平均。路径点参与满窗口后固定，仅更新后方未稳定路线。1 表示仅使用新路线。 |
| 路线平均点距 | `history_spacing` | 1.0 格 | 0.5–3.0 格 | 0.1 | 沿未稳定的新路线按弧长取对应点的间隔。 |

## 安全预瞄

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 最短可用路线 | `min_route` | 4.0 格 | 2.0–12.0 格 | 0.5 | 比此值更短的路线不用于推进船只。 |
| 最短检查距离 | `safe_distance` | 5.0 格 | 3.0–12.0 格 | 0.5 | 每 tick 检查前方路线的最短范围。 |
| 检查时间范围 | `safe_ticks` | 5.0 tick | 3.0–12.0 tick | 0.5 | 随速度扩大前方检查距离的时间长度。 |
| 转向预瞄时间 | `yaw_lead` | 18.973665961 tick | 8.0–36.0 tick | 0.5 | 提前考虑船头转向惯性的时间长度。 |
| 笔画前视宽度倍数 | `painted_forward_widths` | 4.0 | 1.0–8.0 | 0.25 | 有几何笔画宽度时，用多少倍半宽检查连续弯的方向变化。 |
| 笔画响应宽度倍数 | `painted_lead_widths` | 2.0 | 1.0–4.0 | 0.25 | 有几何笔画宽度时，转向响应距离的基础半宽倍数。 |
| 笔画弯道提前量 | `painted_bend_widths` | 2.0 | 0.0–4.0 | 0.25 | 按连续弯的有符号扫角增加响应距离，反向弯会抵消。 |
| 笔画预瞄放大上限 | `painted_width_factor_max` | 2.0 | 1.0–3.0 | 0.1 | 宽笔画按船宽放大前视与响应距离的上限。 |
| 弯道采样数 | `sweep_samples` | 8.0 | 4.0–24.0 | 1.0 | 计算预瞄内方向变化的采样数量。 |
| 安全点细化次数 | `preview_bisections` | 7.0 | 4.0–12.0 | 1.0 | 直线抄近路不安全时，缩短预瞄距离的二分次数。 |

## 侧滑反馈

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 切弯倾向 | `shortcut_weight` | 0.3 | 0.0–1.0 | 0.05 | 冰面允许时，沿预瞄点适度切向弯内侧。扣除向内侧速度的刹停距离。 |
| 切弯余量采样 | `shortcut_samples` | 4.0 | 2.0–12.0 | 1.0 | 沿前方路线检查内侧冰面余量的次数。 |
| 中心纠偏强度 | `position_gain` | 2.0 | 0.5–5.0 | 0.1 | 偏离中心时，向中心施加的横向修正强度。 |
| 笔画反馈宽度上限 | `painted_feedback_width` | 4.0 格 | 1.0–8.0 格 | 0.25 | 有几何笔画宽度时，限制侧滑反馈时间所用的半宽。 |
| 实测反馈宽度上限 | `free_feedback_width` | 3.0 格 | 1.0–8.0 格 | 0.25 | 仅有实测冰面宽度时，限制侧滑反馈时间，避免宽冰面纠偏过慢。 |
| 宽冰面额外纠偏 | `wide_position_gain` | 2.0 | 0.0–4.0 | 0.1 | 没有几何笔画宽度时，宽冰面增加的位置反馈强度。 |
| 宽冰面纠偏起点 | `wide_position_start` | 4.0 格 | 2.0–8.0 格 | 0.25 | 实测半宽超过此值时，逐渐增加位置反馈。 |
| 宽冰面纠偏过渡 | `wide_position_band` | 4.0 格 | 1.0–8.0 格 | 0.25 | 额外位置反馈从零到最大所经过的半宽范围。 |
| 侧滑阻尼 | `lateral_damping` | 3.0 | 0.5–6.0 | 0.1 | 根据当前横向速度抵消过度侧滑。 |
| 窄道额外阻尼 | `narrow_damping` | 0.5 | 0.0–2.0 | 0.1 | 冰面余量较小时增加的侧滑阻尼。 |
| 旋转响应权重 | `yaw_response` | 1.0 | 0.0–2.0 | 0.1 | 旋转速度较大时，延长横向修正的响应时间。 |
| 侧后转向修正 | `rear_side_reduction` | 0.3 | 0.0–0.8 | 0.05 | 制动转向时降低横向请求，避免过度横移。 |
| 侧边探测距离 | `control_margin` | 6.0 格 | 2.0–12.0 格 | 0.5 | 评估切弯余量时，沿路线法向探测整船冰面支持的最大距离。 |

## 弯道制动

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 制动启用速度 | `brake_min_speed` | 0.05 格/tick | 0.01–0.15 格/tick | 0.01 | 低于此值不请求弯道制动。只用于近静止状态，不是目标速度。 |
| 制动提前时间 | `brake_lead` | 8.0 tick | 2.0–20.0 tick | 0.5 | 在惯性制动距离之外增加的弯道预览时间。 |
| 制动采样间隔 | `brake_step` | 0.5 格 | 0.25–1.0 格 | 0.05 | 弯道制动扫描的最小距离间隔。 |
| 高速采样比例 | `brake_speed_step` | 0.5 | 0.1–1.0 | 0.05 | 随速度扩大弯道扫描间隔，平衡精度与耗时。 |
| 横向推力余量 | `lateral_budget` | 0.9 | 0.3–1.0 | 0.05 | 弯道可用横向推力比例。较低值更早提出制动。 |
| 窄弯推力基础比例 | `hull_budget_base` | 0.6 | 0.2–1.0 | 0.05 | 无额外船体余宽时保留的横向预算比例；余宽增大后平滑恢复。 |
| 窄弯余量比例下限 | `corner_room_min` | 0.7 | 0.3–1.0 | 0.05 | 余宽相对船宽缩小时，弯道横向预算保留的最低比例。 |
| 急弯敏感度 | `rapidity_weight` | 0.5 | 0.0–5.0 | 0.1 | 曲率突然变化时增加制动请求。 |
| 反向弯敏感度 | `reversal_weight` | 0.35 | 0.0–3.0 | 0.05 | 连续反向弯出现时增加制动请求。 |
| 制动距离下限 | `brake_min_distance` | 2.0 格 | 1.0–6.0 格 | 0.25 | 计算所需制动强度时使用的最小剩余距离。 |
| 制动距离时间下限 | `brake_min_ticks` | 6.0 tick | 2.0–12.0 tick | 0.5 | 以当前速度乘此时间设置可用制动距离下限，避免近弯请求突变。 |
| 侧后转向起点 | `rear_start` | 0.9 格/tick | 0.3–2.0 格/tick | 0.1 | 有制动需求且超过此速度时，逐渐允许向侧后方转向。 |
| 侧后转向过渡 | `rear_band` | 1.1 格/tick | 0.2–2.0 格/tick | 0.1 | 侧后方转向权重随速度增加的过渡范围。 |

## 转向控制

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 路线方向权重 | `guide_weight` | 2.7 | 0.0–6.0 | 0.1 | 期望船头方向求解时，对预瞄路线方向的重视程度。 |
| 转向代价权重 | `heading_cost` | 1.0 | 0.1–3.0 | 0.1 | 权衡推力方向与船头大幅旋转的代价。 |
| 窄道判定半宽 | `edge_width` | 2.5 格 | 1.0–5.0 格 | 0.1 | 半宽低于此值时延长转向响应并增加反打余量，不限制直道速度。 |
| 路线参考角速度上限 | `reference_rate_limit` | 5.0°/tick | 1.0–10.0°/tick | 0.25 | 限制曲率与有符号沿线速度给出的参考角速度，防止尖峰造成旋转。 |
| 角度响应时间 | `yaw_response_ticks` | 5.0 tick | 2.0–12.0 tick | 0.5 | 小航向误差转换为角速度修正的时间。较大误差受刹停角约束。 |
| 窄道响应延长 | `edge_response_ticks` | 2.0 tick | 0.0–6.0 tick | 0.5 | 窄冰面上增加的角度响应时间，减小连续反向转舵。 |
| 角速度死区 | `rate_deadzone` | 0.8°/tick | 0.1–2.0°/tick | 0.1 | 目标角速度与实测阻尼后角速度的允许误差。中性 tick 保留旋转惯性。 |
| 提前反打强度 | `stop_gain` | 1.0 | 0.25–2.0 | 0.05 | 限制大航向误差的角速度修正，使现有旋转可提前反打制止。 |
| 冰边反打余量 | `edge_stop_gain` | 0.3 | 0.0–1.0 | 0.05 | 接近窄冰面时增加提前反打的强度。 |
| 停机旋转死区 | `stop_deadzone` | 0.5°/tick | 0.1–2.0°/tick | 0.1 | 没有路线时，允许保留的微小角速度。 |

## 起步与求解

| Control | JSON key | Default | Range | Step | Effect |
| --- | --- | --- | --- | --- | --- |
| 笔画滑行制动速度 | `coast_brake_speed` | 1.5 格/tick | 0.3–3.0 格/tick | 0.1 | 笔画路线上低于此速度且误差很小时，可抑制 S 交给天然阻尼制动。转向仍保留。 |
| 笔画滑行冰边风险 | `coast_brake_risk` | 0.5 | 0.1–0.9 | 0.05 | 笔画滑行抑制 S 的冰边风险上限。 |
| 笔画滑行角差 | `coast_brake_angle` | 8.0° | 2.0–20.0° | 0.5 | 笔画滑行抑制 S 时允许的最大航向误差。 |
| 笔画滑行旋转速度 | `coast_brake_rate` | 3.0°/tick | 0.5–6.0°/tick | 0.25 | 笔画滑行抑制 S 时允许的最大实测角速度。 |
| 低速反推分界 | `reverse_low_speed` | 0.55 格/tick | 0.1–1.0 格/tick | 0.05 | 仅选择 S 脉冲的释放周期，不作为前进速度限制。 |
| 低速反推周期 | `reverse_slow_period` | 10.0 tick | 5.0–20.0 tick | 1.0 | 低速每个周期至少释放一次 S。释放时 A/D 仍会产生原版裸转向推力。 |
| 常速反推周期 | `reverse_fast_period` | 5.0 tick | 2.0–20.0 tick | 1.0 | 较高速度每个周期至少释放一次 S，避免长时间占用按键。 |
| 方向求解精度 | `solver_iterations` | 14.0 | 6.0–24.0 | 1.0 | 选择连续推力方向时的搜索次数。 |
| 累计转向预测 | `forecast_ticks` | 28.0 tick | 8.0–48.0 tick | 1.0 | 累计中间船头方向所贡献的前进推力的最长时间。 |
| 最短转向预测 | `forecast_min` | 3.0 tick | 1.0–8.0 tick | 1.0 | 至少累计多少 tick 后才能结束转向预测。 |
| 预测结束角差 | `forecast_angle` | 2.0° | 0.5–5.0° | 0.1 | 预测船头接近期望角度到此程度时，可以结束累计。 |
