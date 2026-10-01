---
name: icer
description: >
  Icer: Minecraft 26.2 水平冰面冰船自动驾驶控制器的独立算法规范。涉及路线跟踪、贝塞尔曲线、侧滑、冰面安全预瞄、预弯制动、累计转向推力、WASD 决策和角速度反打时使用；适用于跨项目复现、移植和调试。非汽车转向模型。
---

# Icer — Minecraft 26.2 冰船自动驾驶

Icer 是逐 tick 反馈的路线跟踪与逆动力学按键控制器：投影到路线 → 选冰面安全预瞄点 p → 根据弯道与侧滑选择船头及 W/S 推力 → 累计转向途中每个船头角的侧向冲量 → 预制动角速度并输出 W/A/S/D。没有人为设定的最大速度或目标速度；直道可持续 W 加速，弯道通过旋转船头及推力方向调节速度和侧滑。它不是轨迹搜索、MPPI 或汽车模型。经验系数需在目标赛道验证。

## 1. 状态、坐标及真实动力学

世界平面 `(x,z)`；路线切向 `T=(tx,tz)`，左法向 `N=(-tz,tx)`；航向 `yaw` 单位度，船头 `H(yaw)=(-sin(yaw),cos(yaw))`；速度 `v=(vx,vz)` 是世界矢量，不随船头旋转；`omega` 为度/tick 的角速度。路线弧长 `s` 单位格，曲率 `κ=(x'z''-z'x'')/(x'^2+z'^2)^(3/2)`，其符号与上述法向一致。速度单位格/tick，加速度单位格/tick²。`wrap(a-b)=((a-b+540)%360+360)%360-180` 为最短有符号角差。

每个整 tick 的水平冰船动力学顺序：

```text
F = 船体底部接触方块的平均摩擦系数          // 在本 tick 位置先采样
v' = F v                                    // 世界速度整体阻尼，绝无横向抓地力
ω' = float32(float32(ω)*F) + (D?1:0) - (A?1:0)  // 每次加减均 float32
ψ' = float32(float32(ψ)+ω')                 // 先改变船头角
u = (A xor D 且 !W 且 !S ? .005 : 0) + (W ? .04 : 0) - (S ? .005 : 0)
v' += u H(ψ')                              // 沿新船头方向施力，不旋转旧速度
(x',z') = (x,z)+v'
```

为贴近 26.2 原版精度，yaw/omega/加速度按单精度浮点舍入；三角函数使用 Minecraft 65536 项正弦表（`trunc(angle*10430.378350470453)&65535`，余弦偏移 16384）；位置与平面速度按双精度计算。标准冰/浮冰 `F=.98`，蓝冰 `.989`，普通地面 `.6`。宽 1.375 格的船体底面与平铺的整格方块相交：每个触及的块计一次，**算术平均摩擦系数，不按重叠面积加权**。这里的地面/平面模型不模拟水、竖直运动、碰撞及实体，不能声称实船所有状态都被覆盖。持续 W 直驶满足 `v_next=F*v+.04`，因此施力和移动后的稳态速度 `v∞=.04/(1-F)`：冰约 2.0，蓝冰约 3.636 格/tick；若观测位置在本 tick 阻尼后、控制施力前，则该状态的稳态速度为 `F*v∞=.04F/(1-F)`，冰约 1.96、蓝冰约 3.596。复现时必须说明状态采样时点，不能把后一速度当作移动阶段的终端速度。松开 A/D 只阻尼旋转，不立刻停止。

冰面判定要求船体覆盖的每块都是普通冰/浮冰/蓝冰；两点连线的冰面可行性每最多 .5 格采样整段船体，用于检查几何抄近路，属于离散近似而非连续碰撞证明。

## 2. 路线几何及投影

路线为首尾连接的三次 Bézier 段 `B(t)=(1-t)^3P0+3(1-t)^2tP1+3(1-t)t²P2+t³P3`。一阶导数 `3(1-t)²(P1-P0)+6(1-t)t(P2-P1)+3t²(P3-P2)`；二阶导数 `6(1-t)(P2-2P1+P0)+6t(P3-2P2+P1)`。每段先按控制多边形长度以 8 样本/格采样（步数限制到 16..4000），累加折线长度成弧长表；弧长查询时二分弧长、插值参数 t，再用解析曲线/导数求点、切向、曲率。小控制柄拼接处（相邻柄长较小，`span=2b=1.375`，`b` 为船体半宽）在接缝附近用前后跨船宽的方向、夹角/半跨弧长估曲率，以 `1-(|s-s_joint|/span)^4` 混合，避免病态尖峰。

求船位 `Q=(x,z)` 的最近路线投影：按 Bézier 控制点包围盒下界排序各段，用折线局部投影找初值，然后每段至多 12 次截断 Newton 迭代极小化 `|B(t)-Q|²`，每步 `t←clamp(t-clamp(g/g',-.2,.2),0,1)`，其中 `g=(B-Q)·B'`、`g'=B'·B'+(B-Q)·B''`；与端点比较，距离相同（1e-9）优先靠近上次进度的弧长，避免自交处跳段。结果包含投影点、单位切向、曲率、弧长、距离。

在路线点两侧以 .25 格间隔沿法向探测船体仍在冰上的距离，结合贝塞尔冰面笔画的几何半宽 `h_g`；局部走廊半宽 `h=max(h_g,b+两侧最小实测余量,干地缺口兜底宽)`，其中 `b=1.375/2`。可按半格弧长区间缓存，但改冰面/路线时必须失效。干地上仍给出非零路线走廊，控制器才会追寻下一块冰。局部宽度仅是探测估计，不能代替全局可通行性校验。

## 3. 冰面安全预瞄

计算当前沿路线速度 `v∥=max(0,v·T)`、半宽 `h`、余宽 `c=max(0,h-b)`、船头响应距离 `Lψ=max(2√90 v∥,2b)`。若有贝塞尔冰面笔画宽 `h_g`：`widthFactor=clamp((h_g-b)/(2b),1,2)`；前视弧长 `L_f=max(Lψ,4h_g)*widthFactor`，否则 `L_f=Lψ`。沿 `L_f` 等间隔取 8 点，对连续路线切向的有符号 `atan2(cross,dot)` 求和为 `signedSweep`；`leadWidths=2+2 clamp(|signedSweep|/π,0,1)`；有笔画时响应距离 `L_r=max(Lψ,leadWidths*h_g)*widthFactor`，否则 `L_r=Lψ`。再在 `L_r` 内取 8 点，累加切向变化绝对角 `angularSweep`；`bend=max(|κ| L_r,angularSweep)`、`bendWeight=bend/(1+bend)`。

设船在弯道内侧偏移 `inside=max(0,sign(κ)*(position-projection)·N)`。请求预瞄距离

```text
L_req = min(routeLength-s, L_r * (有笔画 ? bendWeight : sqrt(bendWeight))
                          * (1+inside/(inside+c+1e-9)))
```

`L_req≈0` 则保持投影点；若从**当前船位**到弧长 `s+L_req` 的路线点的直线船体采样全部在冰上，采用该点；否则对安全距离做 7 次二分，退回最远安全预瞄（没有则投影点）。注意二分隐含“安全性随距离单调”的近似，在复杂非凸冰面不能当作严格安全证明。相反方向连续弯的有符号扫角会抵消，避免预瞄跨过第二个弯。

目标进度 `s_target=max(旧目标进度, 当前投影 s, 本次预瞄 s)` 单调不回退；实际目标 `p=B(s_target)`，其中 `B(s)` 表示按弧长查询的路线点。这与当前投影进度不同，回头/自交路线须专门测试。

## 4. 前瞻制动需求，无最大速度

Icer 不设固定最高速度，也不求目标巡航速度。没有弯道制动需求时，切向推力请求为 `.04`，包括窄直道；不能仅因为局部走廊较窄，就在整段路线限制前进。将来的弯道横向推力不足时，依据所需净减速度与当前冰面天然阻尼降低或反转切向推力。当前侧滑进入横向推力与船头方向的反馈。终端速度只用于航向代价归一化，不是限速。

令 `speed=|v|`、`F` 为当前位置船体接触方块的平均摩擦系数；`speed<.05` 时返回 `B=0`。预览弧长 `L=min(routeLength-s, max(speed²/(2*.04),speed/max(.01,1-F))+8*speed)`，每 `max(.5,speed/2)` 格取样。它同时覆盖强推力制动距离和天然阻尼的长尾。对 `|κ|≥1e-6` 的曲率点，令 `clearance=max(0,h-b)`；只有 `clearance>0`、`|κ|*clearance>1`，且弧长 `s_sample-h` 到 `s_sample+h` 的直线整船在冰上时，才用 `shortcutWeight=clamp(|κ|*clearance-1,0,1)` 把 `|κ|` 向 `1/clearance` 混合，形成正值 `κ_eff`。

以 `L_yaw=sqrt(.04/κ_eff)*2√90` 查询该点前后曲率与更远曲率。令 `change=max(|κ_sample-κ_before|,|κ_after-κ_sample|)`、`rapidity=change/(κ_eff+change)*(1-shortcutWeight)`；`reversal=(|κ_before|+|κ_farAfter|-|κ_before+κ_farAfter|)/(|κ_before|+|κ_farAfter|+1e-9)`，其中远点在 `s_sample+2L_yaw`；`reversalCost=reversal*L_yaw/(L_yaw+2h)`、`hullClearance=max(0,clearance/h)`。累计所需净减速度：

```text
required = 0
cornerRoom = clamp(clearance/(2*b), .7, 1)
a_lat = .04*.9*(.6+.4*hullClearance)*cornerRoom
        /(1+.5*rapidity+.35*reversalCost)
excess = speed²*κ_eff-a_lat
if excess>0:
    coastDistance = (speed-sqrt(a_lat/κ_eff))/max(.01,1-F)
    if distance<=coastDistance+8*speed:
        availableDistance = max(2,6*speed,distance-8*speed)
        required = max(required,excess/(2*κ_eff*availableDistance))
B = required>0 ? clamp((.04+required-speed*(1-F))/.08,0,1) : 0
a_parallel = .04*(1-2*B)
rearTurnWeight = B*clamp((speed-.9)/1.1,0,1)
```

这里 `b=.6875` 为船体半宽。上述局部公式对每个预览弯点计算，`required` 取所有已进入滑行制动窗口的弯点的最大值；仍在窗口之外的弯点不提前压制直道 W。`sqrt(a_lat/κ_eff)` 只用于估计该弯点的滑行距离，不作为每 tick 跟踪的速度目标。存在制动需求且未被 clamp 截断时，`a_parallel=speed*(1-F)-required`：天然阻尼已经贡献减速度，推力只补足剩余需求，不能再次把净减速度全部算给按键。

`B` 是转船头和制动的请求强度，绝不是速度上限或可证最优刹车曲线。`coastDistance` 使用当前 `F` 和固定方向的滑行近似，未预报未来摩擦变化、转向裸推力或完整弯道轨迹；`speed*(1-F)` 也以总速长近似切向阻尼。高速锐角弯可令 `a_parallel<0`，允许船头转到速度方向的侧后方；此时 W 同时制动并积累横向速度。安全性仍须在真实冰面边界检查。

## 5. 每 tick 的路线与侧滑反馈

每整 tick 重算当前位置投影、冰面安全预瞄点 `p`、走廊宽度和 `B`。定义 `v_parallel=v·T`、`v_side=v·N`、`e=(position-projection)·N`。`p` 的法向偏移提出捷径 `requestedShortcut=.3*R*(p-projection)·N`；在当前点及前方四点测该侧可用冰面余量，并扣除已有向内侧速度的刹停距离 `v_in²/(2*.04)`，得 `shortcut` 与 `e_side=e-shortcut`。前方探测范围为 `min(p.s-s,max(0,v_parallel)*2√90)`，若前方路线法向与当前法向的点积不大于零，则停止沿该侧探测。`R` 是可抄近路余量：半宽 `h≥4b` 时取 1；否则向两侧每 .25 格探测整船冰面，最小实测距离记作 `margin`，`R=clamp((margin-2b)/(2b),0,1)`。

```text
v_curve = max(0,v_parallel)
feedbackWidth = 当前位置有贝塞尔冰面笔画 ? 4 : 3
tau = sqrt(2*min(feedbackWidth,h(p))/.04)+abs(omega)*(1-R)
k_d = 3+.5*(1-R)
```

当前曲率与未来 `2√90` tick 经过的 8 个曲率各自 clamp 到 `[-1,1]` 后取九点平均；再用当前切线与 `p` 切线的有符号夹角除以预瞄弧长形成 `κ_preview`，按 `R*previewDistance/(previewDistance+v_curve*2√90+1e-9)` 混合得到 `κ_steer`。实际推力请求直接由未饱和的预览项与反馈项组合：

```text
positionGain = 2 + (当前位置有贝塞尔冰面笔画 ? 0 : 2*clamp((h(projection)-4)/4,0,1))
requestedSide = v_curve²*κ_steer-positionGain*e_side/tau²-k_d*v_side/tau
targetSide = clamp(requestedSide,-.04,.04)         // 组合请求的诊断值
```

不能先截断当前曲率请求再加预览修正，否则病态尖峰被平均曲率替换时，修正量可能把侧向请求错误地反向。反馈时间使用最多 4 格的笔画半宽或 3 格的手绘半宽，手绘宽冰面还增加位置反馈，防止宽度测量很大时允许长时间侧滑偏离路线。`p` 的方向还通过捷径、预瞄曲率和目标切线参与决策；不能只看船头与当前路线切线。

W 每 tick 只能沿船头提供固定 `.04` 的一个矢量。先以 `thrustScale=min(1,.04/hypot(a_parallel,requestedSide))` 约束组合请求；零请求时取 1。令 `outwardSpeed=max(0,v_side*sign(e_side))`、`edgeRisk=clamp((|e_side|+outwardSpeed*tau)/max(.25,h(projection)-b),0,1)`、`sidePriority=clamp(requestedSide,-.04,.04)`、`sidePriorityWeight=max(1-R,当前位置有贝塞尔冰面笔画 ? 0 : edgeRisk)`，然后：

```text
steeringSide = (sidePriority*sidePriorityWeight
               +requestedSide*thrustScale*(1-sidePriorityWeight))*(1-.3*rearTurnWeight)
requestedParallel = a_parallel*(sidePriorityWeight+thrustScale*(1-sidePriorityWeight))
```

窄道或手绘冰边附近优先保留侧向分量，其余位置同比缩放；`steeringSide` 的最后一项避免向侧后方转船头时过度加大侧向请求。冰边风险是反馈权重，不能替代整船冰面检查或连续安全证明。

## 6. 连续推力方向、离散转向和累计 W 求解

在 `u_parallel∈[-sqrt(.04²-steeringSide²),+sqrt(...)]` 上用两半区黄金分割搜索，每半区 14 次，另比较零点和两端点；候选船头为 `tangentYaw+atan2(steeringSide,u_parallel)*180/π`。零矢量退回 `forwardYaw=tangentYaw+asin(clamp(steeringSide/.04,-1,1))*180/π`。代价为 `(u_parallel-requestedParallel)²+yawCost(候选船头)`；航向代价同时惩罚相对当前船头和路线引导角的转角，`guideYaw=tangentYaw+wrap(pYaw-tangentYaw)*R`，路径权重 `2.7*(1-rearTurnWeight)`。航向代价中的角差均转为弧度，系数为 `turnCost=.04²/π²*(1+speed/(speed+.04F/(1-F)))`；这里沿用阻尼后、控制前的稳态速度尺度作归一化，不把它当作实际移动速度的上限。

令 `H_current` 为当前船头；`coastCost=requestedParallel²+steeringSide²+yawCost(forwardYaw)`，`reverseCost=(-.005*H_current·T-requestedParallel)²+(-.005*H_current·N-steeringSide)²+yawCost(forwardYaw)`，`useW=bestW.cost<min(coastCost,reverseCost)`。按这些代价选择推力，但 **`targetYaw` 始终使用最佳 W 候选的船头角**，不能在 W 与滑行切换时跳回 `forwardYaw`。S 仅是 W 不适合当前船头时的弱反向推力候选，不能因 `reverseCost<coastCost` 就在整段弯道连续按住。

连续方向和占空随后转换为真实离散按键。用有符号 `v_parallel` 求路线参考角速度，并限制为 `omega_ref=clamp((κ_current+κ_steer)/2*v_parallel*180/π,-5,5)`，其中当前曲率已 clamp 到 `[-1,1]`。令 `edgeRoom=clamp(2.5-h(projection),0,1)`；它只调节转向响应，不另加宽度限速。A/D 跟踪有界目标角速度：

```text
dampedOmega = omega*F
relativeOmega = dampedOmega-omega_ref
yawError = wrap(targetYaw-yaw)
stoppingAngle = relativeOmega*abs(relativeOmega)/2*(1+.3*edgeRoom)
brakeError = yawError-stoppingAngle               // 诊断值，不再直接开关 A/D
correction = sign(yawError)*min(abs(yawError)/(5+2*edgeRoom),
                               sqrt(2*abs(yawError)/(1+.3*edgeRoom)))
omegaError = omega_ref+correction-dampedOmega
turn = omegaError>.8 ? +1 : omegaError<-.8 ? -1 : 0
A = turn<0; D = turn>0
```

小误差按响应时间修正，较大误差受刹停角对应的平方根速度界约束；±.8°/tick 的中性区给角速度阻尼留出 tick，减少半度航向开关造成的连续反向转向。零 `turn` 仍保留旋转惯性，已有角速度超过目标时必须反打；累计求解器与真实按键使用同一个转向判据。

**累计求解器：**从当前 `yaw,omega` 出发，最多预报 28 个整 tick。每一步使用上面同一个 A/D 决策，先按当前冰面 `F` 阻尼角速度，再施加 ±1 度/tick²，再更新船头；用 Minecraft 正弦表求 W 沿当前路线法向的侧向冲量 `side_i=.04*sign(steeringSide)*H(yaw_i)·N`。到目标角差小于 2° 且至少预报 3 tick 后停止。只累计对所需方向有利的 `max(0,side_i)`：

```text
availableSide = sum_i max(0,side_i)
requiredSide = abs(steeringSide)*ticks
turnDuty = firstSide>0 ? clamp(requiredSide/availableSide,0,1) : 0
```

这正是“转角过程也按 W”的求解：若三个中间船头角依次贡献 1、2、3，目标累计侧向冲量为 6，则 `turnDuty=6/(1+2+3)=1`，三个 tick 都按 W，而不是转到末角后才按一次。真实控制还会检查同 tick 的切向需求、制动冲突和 W/S 代价，所以不是所有转向 tick 都保证按 W。

当前 tick 先估计下一船头 `H_next`，把最佳 W 推力矢量投影到它，得到 `projectedRequest`。若 W 在当前速度方向上反而增加前进速度，则以 `B*max(0,H_next·v/|v|)` 抵扣累计占空。W 的占空仍由累计求解器决定：

```text
cumulativeDuty = max(0,turnDuty-B*max(0,H_next·v/|v|))
duty = useW ? clamp(max(projectedRequest/.04,cumulativeDuty),0,1) : 0
phase += duty                 // 仅 duty>0 时；初始 phase=1
W = phase>=1; if W: phase-=1
```

对 S 单独做脉冲决策。`coastBrake` 在 `!useW`、`reverseCost<coastCost`、船位对应 Bézier 冰面笔画、`speed<1.5`、`edgeRisk<.5`、`|yawError|<8°`、`|omega|<3°/tick` 时成立。它只抑制 S，使天然阻尼承担更多减速，**所有状态仍保留上述 A/D 转向反馈**。其余状态按以下规则打断连续 S：

```text
period = speed<=.55 ? 10 : 5
S = !W && !coastBrake && reverseCost<coastCost
    && tick%period != period-1
```

这里 `tick` 是当前状态的整 tick 编号。较高速度每 5 tick 至少释放一次 S，低速每 10 tick 释放一次，避免小幅反向推力长时间占用按键；低速窄弯因此仍能较密集地制动。**释放 S 不等于零推力**：包括 `coastBrake` 在内，只要 A/D 仍按着且没有 W/S，原版动力学就额外给船头方向 `.005` 前推力；只有 W/A/S/D 全部释放才是无按键推力的被动滑行。不能忽略这项推力或把所有 S 换成 A/D 空转。

`duty` 是 W 的脉冲占空，不是单 tick 连续推力。预报使用当前 `F` 和固定目标角/参考角速度，下一真实 tick 必须从新观测状态重算；它不是开环轨迹规划。判断中间角的横向贡献时要使用**转向后的船头**，与第 1 节动力学顺序一致。调整 S 节奏时同时检查 S 总 tick、最长连续 S、终点 tick、路线误差和整船是否留在冰面；不能只看按键次数。

## 7. 执行时序及验证边界

自动驾驶从路线起点、路线切向角、零速度重置船与占空相位；立即计算首次决策。每 tick 拆为五个 0.2 tick 输入单位，累计五个单位后才推进一次动力学，再更新投影、终点条件和下一整 tick 的按键。回放离散 W/A/S/D 序列，不能用连续占空代替真实按键。终点判定还要检查路线进度、终点附近距离和越过终点切向平面；停机不保证船静止。

复现或修改时优先核对三个不变量：世界速度不随船头转；每 tick 先按接触方块平均摩擦阻尼，再转船头、施力、移动；角速度需预测反打。保持贝塞尔路线、整船足迹、冰面安全预瞄和累计 W 判据一致。端点、自交、连续反向弯、干地缺口和窄道要分别测。当前控制器的回归在部分手绘连续弯上仍可能偏离中心线，仿真结果也不等于 Minecraft 服务器含碰撞、水面、上下坡与校正的实测表现。

按键比例以实际执行的整 tick 统计：`n_W,n_A,n_S,n_D` 分别为各键被按住的 tick 数，`keyTotal=n_W+n_A+n_S+n_D`；同一 tick 的 W+D 计两次按键。报告 `W_share=n_W/keyTotal`、`S_share=n_S/keyTotal`，同时单独报告 `W_duty=n_W/总执行tick`、`S_duty=n_S/总执行tick`。可将代表性赛道汇总后的 `W_share>60%`、`S_share<20%` 作为前进积极性的回归目标，不能用强制配额或周期性补 W 来达到比例；必要制动和整船冰面覆盖优先。分别核对所有赛道是否到终点、整船离冰 tick、中心落地 tick、最大/RMS 路线误差、总转角、A/D 换向和相邻 tick 反打次数；零抓地力、裸转向推力、float32 船头更新及真实离散按键必须保留。已有干地缺口的地图应与修改前的同地图结果比较，不能把路线本身不全在冰上误报成控制器已保证全程冰面。
