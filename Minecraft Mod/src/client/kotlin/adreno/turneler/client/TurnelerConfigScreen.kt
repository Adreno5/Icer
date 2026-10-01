package adreno.turneler.client

import adreno.turneler.navigation.IcerParameter
import adreno.turneler.navigation.IcerSettings
import net.minecraft.client.Minecraft
import net.minecraft.client.gui.GuiGraphicsExtractor
import net.minecraft.client.gui.components.events.GuiEventListener
import net.minecraft.client.gui.screens.Screen
import net.minecraft.client.input.KeyEvent
import net.minecraft.client.input.MouseButtonEvent
import net.minecraft.network.chat.Component
import org.lwjgl.glfw.GLFW
import kotlin.math.roundToInt

/** Native grouped settings with progressive disclosure and a separate, unobstructed HUD placement view. */
class TurnelerConfigScreen(private val client: Minecraft, private val config: TurnelerConfig) :
    Screen(Component.literal("Turneler")) {
    private enum class Page(val key: String, val label: String) {
        DRIVE("drive", "驾驶"), TUNING("tuning", "调校"), DISPLAY("display", "显示")
    }
    private val view = config.settingsView.sanitize()
    private var page = Page.entries.first { it.key == view.page }
    private var tuningGroup: String? = if (page == Page.TUNING) view.group else null
    private var panelLeft = 0
    private var panelTop = 0
    private var panelWidth = 0
    private var panelHeight = 0
    private var scroll = view.scrollFor(page.key, tuningGroup)
    private var positioning = false
    private var hudDragging = false
    private var barDragging = false
    private var hudGrabX = 0.0
    private var hudGrabY = 0.0
    private val rows = arrayListOf<TurnelerWidget>()
    private val chrome = arrayListOf<TurnelerWidget>()
    private val viewportTop get() = panelTop + if (panelHeight < 300) 86 else 94
    private val viewportBottom get() = panelTop + panelHeight - 32
    private val viewportHeight get() = (viewportBottom - viewportTop).coerceAtLeast(1)
    private val maxScroll get() = (rows.size * rowHeight + 12 - viewportHeight).coerceAtLeast(0)
    private val contentLeft get() = panelLeft + 28
    private val contentWidth get() = panelWidth - 56

    override fun init() {
        clearWidgets()
        rows.clear()
        chrome.clear()
        panelWidth = minOf(500, width - 20).coerceAtLeast(180)
        panelHeight = minOf(492, height - 20).coerceAtLeast(180)
        panelLeft = (width - panelWidth) / 2
        panelTop = (height - panelHeight) / 2
        if (positioning) {
            return
        }
        if (page == Page.TUNING && tuningGroup != null) {
            addChrome(TurnelerActionButton(contentLeft, panelTop + 61, 48, 24, "‹ 返回", font) {
                navigate(Page.TUNING, null)
            })
        } else {
            val tabWidth = (panelWidth - 48) / 3
            Page.entries.forEachIndexed { index, target ->
                addChrome(TurnelerTabButton(panelLeft + 24 + index * tabWidth, panelTop + 60, tabWidth, 26,
                    target.label, font, { page == target }) {
                    navigate(target, if (target == Page.TUNING) view.group else null)
                })
            }
        }
        when (page) {
            Page.DRIVE -> {
                toggle("自动驾驶", "乘坐冰船后，自动跟随前方的冰面路线。", { config.enabled }) { config.enabled = it }
                slider("路线视野", "向前扫描的路线长度。", 24.0, 72.0, 1.0,
                    { config.icerHorizon.toDouble() }, { config.icerHorizon = it.roundToInt() }, " 格")
            }
            Page.TUNING -> {
                val group = tuningGroup
                if (group == null) {
                    IcerParameter.entries.groupBy { it.group }.forEach { (name, parameters) ->
                        navigation(name, "${parameters.size} 项可调参数") {
                            navigate(Page.TUNING, name)
                        }
                    }
                } else {
                    IcerParameter.entries.filter { it.group == group }.forEach { p ->
                        slider(p.label, p.description, p.minimum, p.maximum, p.step,
                            { p.sanitize(config.parameters[p.key]) },
                            { config.parameters = IcerSettings(config.parameters + (p.key to p.sanitize(it))).toMap() }, p.unit)
                    }
                }
            }
            Page.DISPLAY -> {
                toggle("路线预览", "在世界中显示路线与当前预瞄点。", { config.showPrediction }) { config.showPrediction = it }
                toggle("透过方块显示", "被地形遮挡时，仍显示路线标记。", { config.alwaysOnTop }) { config.alwaysOnTop = it }
                toggle("状态栏", "显示当前状态、速度和按键。", { !config.hudHidden }) { config.hudHidden = !it }
                val size = TurnelerHud.measure(font, TurnelerClient.pilot.hudLines)
                slider("水平位置", "状态栏距屏幕左侧的距离。", 0.0, maxOf(1, width - size[0]).toDouble(), 1.0,
                    { config.hudX.toDouble() }, { config.hudX = it.roundToInt() }, " px")
                slider("垂直位置", "状态栏距屏幕顶部的距离。", 0.0, maxOf(1, height - size[1]).toDouble(), 1.0,
                    { config.hudY.toDouble() }, { config.hudY = it.roundToInt() }, " px")
                navigation("拖动调整位置", "拖动状态栏，或用方向键微调。") {
                    config.hudHidden = false
                    positioning = true
                    rebuildWidgets()
                }
            }
        }
        addChrome(TurnelerActionButton(contentLeft, panelTop + panelHeight - 30, 86, 22, "恢复默认", font) { resetPage() })
        positionRows()
    }

    private fun addChrome(widget: TurnelerWidget) { chrome += addRenderableWidget(widget) }
    private fun addRow(widget: TurnelerWidget) { rows += addRenderableWidget(widget) }
    private fun toggle(label: String, description: String, read: () -> Boolean, write: (Boolean) -> Unit) {
        addRow(TurnelerToggle(contentLeft, 0, contentWidth, rowHeight, label, font, read, write, description))
    }
    private fun slider(label: String, description: String, min: Double, max: Double, step: Double,
        read: () -> Double, write: (Double) -> Unit, unit: String) {
        addRow(TurnelerSlider(contentLeft, 0, contentWidth, rowHeight, label, font, min, max, step, read, write, description, unit))
    }
    private fun navigation(label: String, description: String, action: () -> Unit) {
        addRow(object : TurnelerWidget(contentLeft, 0, contentWidth, rowHeight, label, font, description) {
            override fun activate() = action()
            override fun extractWidgetRenderState(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int, delta: Float) {
                base(g)
                labels(g, width - 24)
                textRight(g, "›", x + width - 2, y + 20, TurnelerTheme.DIM)
            }
        })
    }

    private fun resetPage() {
        when (page) {
            Page.DRIVE -> { config.enabled = true; config.icerHorizon = 48 }
            Page.DISPLAY -> {
                config.showPrediction = true; config.alwaysOnTop = true
                config.hudHidden = false; config.hudX = 6; config.hudY = 6
            }
            Page.TUNING -> {
                val defaults = IcerParameter.entries.filter { tuningGroup == null || it.group == tuningGroup }
                config.parameters = config.parameters + defaults.associate { it.key to it.default }
            }
        }
        config.sanitize()
        rebuildWidgets()
    }

    private fun positionRows() {
        scroll = scroll.coerceIn(0, maxScroll)
        rows.forEachIndexed { index, row -> row.y = viewportTop + 6 + index * rowHeight - scroll }
        rememberView()
    }

    private fun rememberView() = view.remember(page.key, tuningGroup, scroll)

    private fun navigate(target: Page, group: String?) {
        rememberView()
        page = target
        tuningGroup = group
        scroll = view.scrollFor(target.key, group)
        rememberView()
        rebuildWidgets()
    }

    override fun setFocused(listener: GuiEventListener?) {
        super.setFocused(listener)
        val row = listener as? TurnelerWidget ?: return
        if (row !in rows) return
        if (row.y < viewportTop + 6) scroll -= viewportTop + 6 - row.y
        if (row.bottom > viewportBottom - 6) scroll += row.bottom - viewportBottom + 6
        positionRows()
    }

    override fun keyPressed(event: KeyEvent): Boolean {
        if (positioning) {
            when (event.key()) {
                GLFW.GLFW_KEY_ESCAPE, GLFW.GLFW_KEY_ENTER -> endPositioning()
                GLFW.GLFW_KEY_LEFT -> config.hudX = maxOf(0, config.hudX - 1)
                GLFW.GLFW_KEY_RIGHT -> config.hudX++
                GLFW.GLFW_KEY_UP -> config.hudY = maxOf(0, config.hudY - 1)
                GLFW.GLFW_KEY_DOWN -> config.hudY++
                else -> return super.keyPressed(event)
            }
            clampHud()
            return true
        }
        return super.keyPressed(event)
    }

    private fun inViewport(x: Double, y: Double) = x >= contentLeft - 12 &&
        x < contentLeft + contentWidth + 12 && y >= viewportTop && y < viewportBottom

    override fun mouseScrolled(x: Double, y: Double, horizontal: Double, vertical: Double): Boolean {
        if (positioning || !inViewport(x, y)) return false
        if (isDragging) return true
        scroll -= (vertical * rowHeight).roundToInt()
        positionRows()
        return true
    }

    override fun mouseClicked(event: MouseButtonEvent, doubleClick: Boolean): Boolean {
        if (positioning) {
            if (chrome.any { it.isMouseOver(event.x(), event.y()) }) return super.mouseClicked(event, doubleClick)
            if (event.button() == 0 && TurnelerHud.contains(font, config.hudX, config.hudY,
                    TurnelerClient.pilot.hudLines, event.x(), event.y())) {
                clampHud()
                hudDragging = true
                hudGrabX = event.x() - config.hudX
                hudGrabY = event.y() - config.hudY
                return true
            }
            return false
        }
        if (event.button() == 0 && maxScroll > 0 && event.x() >= panelLeft + panelWidth - 14 &&
            event.x() < panelLeft + panelWidth - 4 && event.y() >= viewportTop && event.y() < viewportBottom) {
            barDragging = true
            scrollBar(event.y())
            return true
        }
        if (!inViewport(event.x(), event.y())) {
            // Hidden rows remain keyboard accessible but never intercept clicks on the header/footer.
            val widget = chrome.firstOrNull { it.isMouseOver(event.x(), event.y()) } ?: return false
            val handled = widget.mouseClicked(event, doubleClick)
            if (handled && widget in chrome) { setFocused(widget); setDragging(event.button() == 0) }
            return handled
        }
        return super.mouseClicked(event, doubleClick)
    }

    private fun scrollBar(y: Double) {
        val thumb = maxOf(18, viewportHeight * viewportHeight / (rows.size * rowHeight + 12))
        scroll = (((y - viewportTop - thumb * 0.5) / maxOf(1, viewportHeight - thumb)) * maxScroll).roundToInt()
        positionRows()
    }

    override fun mouseDragged(event: MouseButtonEvent, dx: Double, dy: Double): Boolean {
        if (hudDragging) {
            config.hudX = (event.x() - hudGrabX).roundToInt()
            config.hudY = (event.y() - hudGrabY).roundToInt()
            clampHud()
            return true
        }
        if (barDragging) { scrollBar(event.y()); return true }
        return super.mouseDragged(event, dx, dy)
    }

    override fun mouseReleased(event: MouseButtonEvent): Boolean {
        val handled = hudDragging || barDragging
        hudDragging = false
        barDragging = false
        return super.mouseReleased(event) || handled
    }

    private fun clampHud() {
        val corner = TurnelerHud.placed(config.hudX, config.hudY, TurnelerHud.measure(font, TurnelerClient.pilot.hudLines))
        config.hudX = corner[0]; config.hudY = corner[1]
    }
    private fun endPositioning() { hudDragging = false; positioning = false; clampHud(); rebuildWidgets() }

    override fun onClose() {
        if (positioning) { endPositioning(); return }
        rememberView()
        config.save()
        client.gui.setScreen(null)
    }
    override fun removed() { rememberView(); config.save(); super.removed() }

    override fun extractBackground(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int, delta: Float) {
        if (!positioning) extractBlurredBackground(g)
        g.fill(0, 0, width, height, if (positioning) 0x28000000 else 0x48000000)
    }

    override fun extractRenderState(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int, delta: Float) {
        if (positioning) {
            TurnelerHud.draw(g, font, config.hudX, config.hudY, TurnelerClient.pilot.hudLines)
            chrome.forEach { it.extractRenderState(g, mouseX, mouseY, delta) }
            return
        }
        TurnelerTheme.roundedFill(g, panelLeft - 1, panelTop + 3, panelWidth + 2, panelHeight + 2, 14, 0x24000000)
        TurnelerTheme.roundedFill(g, panelLeft, panelTop, panelWidth, panelHeight, 14, TurnelerTheme.SURFACE)
        val heading = tuningGroup ?: "Turneler"
        g.pose().pushMatrix()
        g.pose().translate(contentLeft.toFloat(), (panelTop + 20).toFloat())
        g.pose().scale(1.6f, 1.6f)
        g.text(font, heading, 0, 0, TurnelerTheme.TEXT, false)
        g.pose().popMatrix()
        if (tuningGroup == null) g.text(font, "冰船自动驾驶", contentLeft, panelTop + 43, TurnelerTheme.MUTED, false)
        if (tuningGroup == null) TurnelerTheme.roundedFill(g, panelLeft + 22, panelTop + 60, panelWidth - 44, 26, 8, TurnelerTheme.LINE)
        chrome.forEach { it.extractRenderState(g, mouseX, mouseY, delta) }
        g.enableScissor(panelLeft + 14, viewportTop, panelLeft + panelWidth - 14, viewportBottom)
        TurnelerTheme.roundedFill(g, contentLeft - 12, viewportTop + 6 - scroll,
            contentWidth + 24, rows.size * rowHeight, 10, TurnelerTheme.CELL)
        rows.forEachIndexed { index, row ->
            if (index > 0) g.fill(contentLeft, row.y, contentLeft + contentWidth, row.y + 1, TurnelerTheme.LINE)
            row.extractRenderState(g, mouseX, mouseY, delta)
        }
        g.disableScissor()
        if (maxScroll > 0) {
            val thumb = maxOf(18, viewportHeight * viewportHeight / (rows.size * rowHeight + 12))
            val y = viewportTop + (viewportHeight - thumb) * scroll / maxScroll
            TurnelerTheme.roundedFill(g, panelLeft + panelWidth - 10, y, 3, thumb, 1, TurnelerTheme.DIM)
        }
    }

    private val rowHeight get() = if (page == Page.DRIVE) 44 else 52
}
