package adreno.turneler.navigation

/** Tunable heuristics. Vanilla thrust, yaw acceleration and the hull footprint are never settings. */
enum class IcerParameter(
    val group: String, val label: String, val description: String,
    val default: Double, val minimum: Double, val maximum: Double, val step: Double,
    val unit: String = "",
) {
    COARSE_STEP("路线生成", "扫描步长", "常规路线扫描的前进距离。减小可提高精度。", 3.0, 0.75, 6.0, 0.25, " 格"),
    FINE_STEP("路线生成", "精细扫描步长", "常规扫描被截断时，用更短的步长重试。", 1.5, 0.25, 3.0, 0.25, " 格"),
    BEAM("路线生成", "候选路线数", "每轮保留的路线数。越大越容易发现弯道，也越耗时。", 12.0, 4.0, 32.0, 1.0),
    TURN_RANGE("路线生成", "扫描转角", "每一步向左右探索的最大角度。", 35.0, 10.0, 65.0, 1.0, "°"),
    WIDTH_LIMIT("路线生成", "两侧探测范围", "测量冰面宽度时，向每侧探测的最大距离。", 8.0, 2.0, 16.0, 0.5, " 格"),
    START_SEARCH("路线生成", "起点搜索范围", "在船身两侧寻找可通行起点的范围。", 3.0, 0.5, 8.0, 0.25, " 格"),
    SECTION_SEARCH("路线生成", "中心搜索范围", "每个扫描点向两侧寻找安全中心的范围。", 2.5, 0.5, 6.0, 0.25, " 格"),
    CENTRE_SHIFT("路线生成", "中心修正上限", "单个扫描点允许向冰面中心移动的距离。", 2.5, 0.25, 4.0, 0.25, " 格"),
    GUIDE_DISTANCE("路线生成", "旧路线引导距离", "沿上一条路线向前取方向，减少新路线抖动。", 5.0, 1.0, 12.0, 0.5, " 格"),
    INCUMBENT_DISTANCE("路线生成", "旧路线有效距离", "船与旧路线的距离超过此值时，重新探索方向。", 4.0, 1.0, 8.0, 0.5, " 格"),
    TRAVEL_THRESHOLD("路线生成", "速度方向启用值", "低于此速度时，扫描方向优先使用船头方向。", 0.18, 0.05, 0.8, 0.01, " 格/tick"),
    MARGIN_WEIGHT("路线评分", "宽度偏好", "更偏好两侧留有余量的路线。", 0.55, 0.0, 2.0, 0.05),
    TURN_WEIGHT("路线评分", "转弯代价", "增大后更偏好平缓的路线。", 0.32, 0.0, 2.0, 0.01),
    SHIFT_WEIGHT("路线评分", "横向修正代价", "降低单步中心位置大幅改变的倾向。", 0.10, 0.0, 1.0, 0.01),
    INCUMBENT_WEIGHT("路线评分", "路线连续性", "增大后更偏好与上一条路线接近的候选。", 0.07, 0.0, 1.0, 0.01),
    SMOOTH_PASSES("曲线拟合", "平滑次数", "滤除方块边缘带来的中心线抖动。窄弯会自动减少平滑。", 8.0, 0.0, 16.0, 1.0),
    ANCHOR_STRIDE("曲线拟合", "锚点间隔", "每隔多少个扫描点拟合一个曲线锚点。", 4.0, 1.0, 8.0, 1.0),
    SMOOTH_ROOM("曲线拟合", "平滑余量比例", "曲线允许偏离扫描中心的距离与冰面余量之比。", 0.45, 0.1, 1.0, 0.05),
    SMOOTH_MIN("曲线拟合", "最小拟合偏移", "曲线拟合允许的最小中心偏差。完整船体检查仍然生效。", 0.25, 0.05, 0.8, 0.05, " 格"),
    SMOOTH_MAX("曲线拟合", "最大拟合偏移", "曲线偏离扫描中心的最大距离。", 1.2, 0.1, 2.5, 0.1, " 格"),
    HANDLE_SCALE("曲线拟合", "曲线控制柄长度", "调节曲线圆滑程度。失败时自动尝试更短控制柄。", 1.0, 0.25, 1.5, 0.05),
    HISTORY_TICKS("曲线拟合", "路线平均窗口", "按真实 tick 对齐原始路线并取平均。路径点参与满窗口后固定，仅更新后方未稳定路线。1 表示仅使用新路线。", 20.0, 1.0, 40.0, 1.0, " tick"),
    HISTORY_SPACING("曲线拟合", "路线平均点距", "沿未稳定的新路线按弧长取对应点的间隔。", 1.0, 0.5, 3.0, 0.1, " 格"),
    MIN_ROUTE("安全预瞄", "最短可用路线", "比此值更短的路线不用于推进船只。", 4.0, 2.0, 12.0, 0.5, " 格"),
    SAFE_DISTANCE("安全预瞄", "最短检查距离", "每 tick 检查前方路线的最短范围。", 5.0, 3.0, 12.0, 0.5, " 格"),
    SAFE_TICKS("安全预瞄", "检查时间范围", "随速度扩大前方检查距离的时间长度。", 5.0, 3.0, 12.0, 0.5, " tick"),
    YAW_LEAD("安全预瞄", "转向预瞄时间", "提前考虑船头转向惯性的时间长度。", 2.0 * kotlin.math.sqrt(90.0), 8.0, 36.0, 0.5, " tick"),
    PAINTED_FORWARD_WIDTHS("安全预瞄", "笔画前视宽度倍数", "有几何笔画宽度时，用多少倍半宽检查连续弯的方向变化。", 4.0, 1.0, 8.0, 0.25),
    PAINTED_LEAD_WIDTHS("安全预瞄", "笔画响应宽度倍数", "有几何笔画宽度时，转向响应距离的基础半宽倍数。", 2.0, 1.0, 4.0, 0.25),
    PAINTED_BEND_WIDTHS("安全预瞄", "笔画弯道提前量", "按连续弯的有符号扫角增加响应距离，反向弯会抵消。", 2.0, 0.0, 4.0, 0.25),
    PAINTED_WIDTH_FACTOR_MAX("安全预瞄", "笔画预瞄放大上限", "宽笔画按船宽放大前视与响应距离的上限。", 2.0, 1.0, 3.0, 0.1),
    SWEEP_SAMPLES("安全预瞄", "弯道采样数", "计算预瞄内方向变化的采样数量。", 8.0, 4.0, 24.0, 1.0),
    PREVIEW_BISECTIONS("安全预瞄", "安全点细化次数", "直线抄近路不安全时，缩短预瞄距离的二分次数。", 7.0, 4.0, 12.0, 1.0),
    SHORTCUT_WEIGHT("侧滑反馈", "切弯倾向", "冰面允许时，沿预瞄点适度切向弯内侧。扣除向内侧速度的刹停距离。", 0.3, 0.0, 1.0, 0.05),
    SHORTCUT_SAMPLES("侧滑反馈", "切弯余量采样", "沿前方路线检查内侧冰面余量的次数。", 4.0, 2.0, 12.0, 1.0),
    POSITION_GAIN("侧滑反馈", "中心纠偏强度", "偏离中心时，向中心施加的横向修正强度。", 2.0, 0.5, 5.0, 0.1),
    PAINTED_FEEDBACK_WIDTH("侧滑反馈", "笔画反馈宽度上限", "有几何笔画宽度时，限制侧滑反馈时间所用的半宽。", 4.0, 1.0, 8.0, 0.25, " 格"),
    FREE_FEEDBACK_WIDTH("侧滑反馈", "实测反馈宽度上限", "仅有实测冰面宽度时，限制侧滑反馈时间，避免宽冰面纠偏过慢。", 3.0, 1.0, 8.0, 0.25, " 格"),
    WIDE_POSITION_GAIN("侧滑反馈", "宽冰面额外纠偏", "没有几何笔画宽度时，宽冰面增加的位置反馈强度。", 2.0, 0.0, 4.0, 0.1),
    WIDE_POSITION_START("侧滑反馈", "宽冰面纠偏起点", "实测半宽超过此值时，逐渐增加位置反馈。", 4.0, 2.0, 8.0, 0.25, " 格"),
    WIDE_POSITION_BAND("侧滑反馈", "宽冰面纠偏过渡", "额外位置反馈从零到最大所经过的半宽范围。", 4.0, 1.0, 8.0, 0.25, " 格"),
    LATERAL_DAMPING("侧滑反馈", "侧滑阻尼", "根据当前横向速度抵消过度侧滑。", 3.0, 0.5, 6.0, 0.1),
    NARROW_DAMPING("侧滑反馈", "窄道额外阻尼", "冰面余量较小时增加的侧滑阻尼。", 0.5, 0.0, 2.0, 0.1),
    YAW_RESPONSE("侧滑反馈", "旋转响应权重", "旋转速度较大时，延长横向修正的响应时间。", 1.0, 0.0, 2.0, 0.1),
    REAR_SIDE_REDUCTION("侧滑反馈", "侧后转向修正", "制动转向时降低横向请求，避免过度横移。", 0.3, 0.0, 0.8, 0.05),
    CONTROL_MARGIN("侧滑反馈", "侧边探测距离", "评估切弯余量时，沿路线法向探测整船冰面支持的最大距离。", 6.0, 2.0, 12.0, 0.5, " 格"),
    BRAKE_MIN_SPEED("弯道制动", "制动启用速度", "低于此值不请求弯道制动。只用于近静止状态，不是目标速度。", 0.05, 0.01, 0.15, 0.01, " 格/tick"),
    BRAKE_LEAD("弯道制动", "制动提前时间", "在惯性制动距离之外增加的弯道预览时间。", 8.0, 2.0, 20.0, 0.5, " tick"),
    BRAKE_STEP("弯道制动", "制动采样间隔", "弯道制动扫描的最小距离间隔。", 0.5, 0.25, 1.0, 0.05, " 格"),
    BRAKE_SPEED_STEP("弯道制动", "高速采样比例", "随速度扩大弯道扫描间隔，平衡精度与耗时。", 0.5, 0.1, 1.0, 0.05),
    LATERAL_BUDGET("弯道制动", "横向推力余量", "弯道可用横向推力比例。较低值更早提出制动。", 0.9, 0.3, 1.0, 0.05),
    HULL_BUDGET_BASE("弯道制动", "窄弯推力基础比例", "无额外船体余宽时保留的横向预算比例；余宽增大后平滑恢复。", 0.6, 0.2, 1.0, 0.05),
    CORNER_ROOM_MIN("弯道制动", "窄弯余量比例下限", "余宽相对船宽缩小时，弯道横向预算保留的最低比例。", 0.7, 0.3, 1.0, 0.05),
    RAPIDITY_WEIGHT("弯道制动", "急弯敏感度", "曲率突然变化时增加制动请求。", 0.5, 0.0, 5.0, 0.1),
    REVERSAL_WEIGHT("弯道制动", "反向弯敏感度", "连续反向弯出现时增加制动请求。", 0.35, 0.0, 3.0, 0.05),
    BRAKE_MIN_DISTANCE("弯道制动", "制动距离下限", "计算所需制动强度时使用的最小剩余距离。", 2.0, 1.0, 6.0, 0.25, " 格"),
    BRAKE_MIN_TICKS("弯道制动", "制动距离时间下限", "以当前速度乘此时间设置可用制动距离下限，避免近弯请求突变。", 6.0, 2.0, 12.0, 0.5, " tick"),
    REAR_START("弯道制动", "侧后转向起点", "有制动需求且超过此速度时，逐渐允许向侧后方转向。", 0.9, 0.3, 2.0, 0.1, " 格/tick"),
    REAR_BAND("弯道制动", "侧后转向过渡", "侧后方转向权重随速度增加的过渡范围。", 1.1, 0.2, 2.0, 0.1, " 格/tick"),
    GUIDE_WEIGHT("转向控制", "路线方向权重", "期望船头方向求解时，对预瞄路线方向的重视程度。", 2.7, 0.0, 6.0, 0.1),
    HEADING_COST("转向控制", "转向代价权重", "权衡推力方向与船头大幅旋转的代价。", 1.0, 0.1, 3.0, 0.1),
    EDGE_WIDTH("转向控制", "窄道判定半宽", "半宽低于此值时延长转向响应并增加反打余量，不限制直道速度。", 2.5, 1.0, 5.0, 0.1, " 格"),
    REFERENCE_RATE_LIMIT("转向控制", "路线参考角速度上限", "限制曲率与有符号沿线速度给出的参考角速度，防止尖峰造成旋转。", 5.0, 1.0, 10.0, 0.25, "°/tick"),
    YAW_RESPONSE_TICKS("转向控制", "角度响应时间", "小航向误差转换为角速度修正的时间。较大误差受刹停角约束。", 5.0, 2.0, 12.0, 0.5, " tick"),
    EDGE_RESPONSE_TICKS("转向控制", "窄道响应延长", "窄冰面上增加的角度响应时间，减小连续反向转舵。", 2.0, 0.0, 6.0, 0.5, " tick"),
    RATE_DEADZONE("转向控制", "角速度死区", "目标角速度与实测阻尼后角速度的允许误差。中性 tick 保留旋转惯性。", 0.8, 0.1, 2.0, 0.1, "°/tick"),
    STOP_GAIN("转向控制", "提前反打强度", "限制大航向误差的角速度修正，使现有旋转可提前反打制止。", 1.0, 0.25, 2.0, 0.05),
    EDGE_STOP_GAIN("转向控制", "冰边反打余量", "接近窄冰面时增加提前反打的强度。", 0.3, 0.0, 1.0, 0.05),
    STOP_DEADZONE("转向控制", "停机旋转死区", "没有路线时，允许保留的微小角速度。", 0.5, 0.1, 2.0, 0.1, "°/tick"),
    COAST_BRAKE_SPEED("起步与求解", "笔画滑行制动速度", "笔画路线上低于此速度且误差很小时，可抑制 S 交给天然阻尼制动。转向仍保留。", 1.5, 0.3, 3.0, 0.1, " 格/tick"),
    COAST_BRAKE_RISK("起步与求解", "笔画滑行冰边风险", "笔画滑行抑制 S 的冰边风险上限。", 0.5, 0.1, 0.9, 0.05),
    COAST_BRAKE_ANGLE("起步与求解", "笔画滑行角差", "笔画滑行抑制 S 时允许的最大航向误差。", 8.0, 2.0, 20.0, 0.5, "°"),
    COAST_BRAKE_RATE("起步与求解", "笔画滑行旋转速度", "笔画滑行抑制 S 时允许的最大实测角速度。", 3.0, 0.5, 6.0, 0.25, "°/tick"),
    REVERSE_LOW_SPEED("起步与求解", "低速反推分界", "仅选择 S 脉冲的释放周期，不作为前进速度限制。", 0.55, 0.1, 1.0, 0.05, " 格/tick"),
    REVERSE_SLOW_PERIOD("起步与求解", "低速反推周期", "低速每个周期至少释放一次 S。释放时 A/D 仍会产生原版裸转向推力。", 10.0, 5.0, 20.0, 1.0, " tick"),
    REVERSE_FAST_PERIOD("起步与求解", "常速反推周期", "较高速度每个周期至少释放一次 S，避免长时间占用按键。", 5.0, 2.0, 20.0, 1.0, " tick"),
    SOLVER_ITERATIONS("起步与求解", "方向求解精度", "选择连续推力方向时的搜索次数。", 14.0, 6.0, 24.0, 1.0),
    FORECAST_TICKS("起步与求解", "累计转向预测", "累计中间船头方向所贡献的前进推力的最长时间。", 28.0, 8.0, 48.0, 1.0, " tick"),
    FORECAST_MIN("起步与求解", "最短转向预测", "至少累计多少 tick 后才能结束转向预测。", 3.0, 1.0, 8.0, 1.0, " tick"),
    FORECAST_ANGLE("起步与求解", "预测结束角差", "预测船头接近期望角度到此程度时，可以结束累计。", 2.0, 0.5, 5.0, 0.1, "°");

    val key: String get() = name.lowercase(java.util.Locale.ROOT)
    fun sanitize(value: Double?): Double {
        val finite = if (value != null && value.isFinite()) value.coerceIn(minimum, maximum) else default
        return if (step == 1.0) kotlin.math.round(finite).coerceIn(minimum, maximum) else finite
    }
}

/** Immutable per-decision snapshot; editing the UI cannot mutate a worker's settings. */
class IcerSettings(values: Map<String, Double> = emptyMap()) {
    private val values = IcerParameter.entries.associateWith { it.sanitize(values[it.key]) }.toMutableMap().apply {
        this[IcerParameter.FINE_STEP] = minOf(getValue(IcerParameter.FINE_STEP), getValue(IcerParameter.COARSE_STEP))
        this[IcerParameter.SMOOTH_MIN] = minOf(getValue(IcerParameter.SMOOTH_MIN), getValue(IcerParameter.SMOOTH_MAX))
        this[IcerParameter.FORECAST_MIN] = minOf(getValue(IcerParameter.FORECAST_MIN), getValue(IcerParameter.FORECAST_TICKS))
        this[IcerParameter.REVERSE_SLOW_PERIOD] = maxOf(getValue(IcerParameter.REVERSE_SLOW_PERIOD), getValue(IcerParameter.REVERSE_FAST_PERIOD))
    }.toMap()

    operator fun get(parameter: IcerParameter): Double = values.getValue(parameter)
    fun int(parameter: IcerParameter): Int = this[parameter].toInt()
    fun toMap(): Map<String, Double> = values.entries.associate { it.key.key to it.value }

    companion object { val DEFAULT = IcerSettings() }
}
