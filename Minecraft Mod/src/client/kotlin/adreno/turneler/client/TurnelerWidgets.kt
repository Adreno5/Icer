package adreno.turneler.client

import net.minecraft.client.gui.Font
import net.minecraft.client.gui.GuiGraphicsExtractor
import net.minecraft.client.gui.components.AbstractWidget
import net.minecraft.client.gui.components.Tooltip
import net.minecraft.client.gui.narration.NarratedElementType
import net.minecraft.client.gui.narration.NarrationElementOutput
import net.minecraft.client.input.KeyEvent
import net.minecraft.client.input.MouseButtonEvent
import net.minecraft.network.chat.Component
import java.util.Locale
import kotlin.math.roundToInt
import kotlin.math.sqrt

/** iOS-style grouped settings: neutral surfaces, dark type, blue actions and green switches. */
internal object TurnelerTheme {
    val SURFACE = 0xFFF2F2F7.toInt()
    val CELL = 0xFFFFFFFF.toInt()
    val HOVER = 0xFFF5F5F8.toInt()
    val LINE = 0xFFE4E4E9.toInt()
    val TEXT = 0xFF1C1C1E.toInt()
    val MUTED = 0xFF6C6C72.toInt()
    val ACCENT = 0xFF007AFF.toInt()
    val DIM = 0xFFAEAEB4.toInt()
    val GREEN = 0xFF34C759.toInt()

    fun roundedFill(g: GuiGraphicsExtractor, x: Int, y: Int, width: Int, height: Int, radius: Int, color: Int) {
        val r = radius.coerceIn(0, minOf(width, height) / 2)
        g.fill(x + r, y, x + width - r, y + height, color)
        g.fill(x, y + r, x + width, y + height - r, color)
        for (row in 0 until r) {
            val dy = r - row - 0.5
            val inset = (r - sqrt(r * r - dy * dy)).roundToInt()
            g.fill(x + inset, y + row, x + width - inset, y + row + 1, color)
            g.fill(x + inset, y + height - row - 1, x + width - inset, y + height - row, color)
        }
    }
}

abstract class TurnelerWidget(
    x: Int, y: Int, width: Int, height: Int, label: String,
    protected val font: Font,
    protected val description: String = "",
) : AbstractWidget(x, y, width, height, Component.literal(label)) {
    protected val labelText = label

    init {
        if (description.isNotEmpty()) setTooltip(Tooltip.create(Component.literal(description)))
    }

    protected fun base(g: GuiGraphicsExtractor) {
        if (isHovered || isFocused) TurnelerTheme.roundedFill(g, x - 5, y + 2, width + 10, height - 4, 5, TurnelerTheme.HOVER)
        if (isFocused) g.fill(x - 5, y + 10, x - 3, bottom - 10, TurnelerTheme.ACCENT)
    }

    protected fun text(g: GuiGraphicsExtractor, value: String, x: Int, y: Int, color: Int) =
        g.text(font, value, x, y, color, false)

    protected fun textRight(g: GuiGraphicsExtractor, value: String, right: Int, y: Int, color: Int) =
        text(g, value, right - font.width(value), y, color)

    protected fun labels(g: GuiGraphicsExtractor, available: Int = width - 80) {
        text(g, font.plainSubstrByWidth(labelText, available), x, y + 12, TurnelerTheme.TEXT)
        text(g, font.plainSubstrByWidth(description, available), x, y + 31, TurnelerTheme.MUTED)
    }

    override fun extractTooltipForNextRenderPass(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int) {
        if (active && visible && g.containsPointInScissor(mouseX, mouseY)) super.extractTooltipForNextRenderPass(g, mouseX, mouseY)
    }

    override fun keyPressed(event: KeyEvent): Boolean {
        if (active && event.isConfirmation()) {
            activate()
            return true
        }
        return super.keyPressed(event)
    }

    override fun onClick(event: MouseButtonEvent, doubleClick: Boolean) {
        if (active && event.button() == 0) activate()
    }

    protected abstract fun activate()
    protected open fun narratedValue(): String = ""

    override fun updateWidgetNarration(output: NarrationElementOutput) {
        output.add(NarratedElementType.TITLE, Component.literal("$labelText ${narratedValue()}"))
        if (description.isNotEmpty()) output.add(NarratedElementType.HINT, Component.literal(description))
    }
}

class TurnelerActionButton(
    x: Int, y: Int, width: Int, height: Int, label: String, font: Font,
    private val action: () -> Unit,
) : TurnelerWidget(x, y, width, height, label, font) {
    override fun activate() = action()
    override fun extractWidgetRenderState(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int, delta: Float) {
        base(g)
        g.text(font, labelText, x + (width - font.width(labelText)) / 2, y + (height - 9) / 2, TurnelerTheme.ACCENT, false)
    }
}

class TurnelerTabButton(
    x: Int, y: Int, width: Int, height: Int, label: String, font: Font,
    private val selected: () -> Boolean,
    private val action: () -> Unit,
) : TurnelerWidget(x, y, width, height, label, font) {
    override fun activate() = action()
    override fun narratedValue() = if (selected()) "已选中" else ""
    override fun extractWidgetRenderState(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int, delta: Float) {
        if (selected()) TurnelerTheme.roundedFill(g, x, y + 2, width, height - 4, 6, TurnelerTheme.CELL)
        if (isFocused) g.outline(x, y + 2, width, height - 4, TurnelerTheme.ACCENT)
        text(g, labelText, x + (width - font.width(labelText)) / 2, y + (height - 9) / 2,
            if (selected() || isHovered) TurnelerTheme.TEXT else TurnelerTheme.MUTED)
    }
}

class TurnelerToggle(
    x: Int, y: Int, width: Int, height: Int, label: String, font: Font,
    private val read: () -> Boolean,
    private val write: (Boolean) -> Unit,
    description: String = "",
) : TurnelerWidget(x, y, width, height, label, font, description) {
    private var switchPosition = if (read()) 1.0 else 0.0
    private var lastFrame = System.nanoTime()
    override fun activate() = write(!read())
    override fun narratedValue() = if (read()) "开启" else "关闭"
    override fun extractWidgetRenderState(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int, delta: Float) {
        base(g)
        labels(g)
        val on = read()
        val now = System.nanoTime()
        val elapsed = ((now - lastFrame) / 1e9).coerceIn(0.0, 0.1)
        lastFrame = now
        switchPosition += ((if (on) 1.0 else 0.0) - switchPosition) * minOf(1.0, elapsed * 18.0)
        val switchX = x + width - 28
        val switchY = y + 16
        TurnelerTheme.roundedFill(g, switchX, switchY, 28, 16, 8, if (on) TurnelerTheme.GREEN else TurnelerTheme.LINE)
        TurnelerTheme.roundedFill(g, switchX + 2 + (switchPosition * 12).roundToInt(), switchY + 2, 12, 12, 6, TurnelerTheme.CELL)
    }
}

class TurnelerSlider(
    x: Int, y: Int, width: Int, height: Int, label: String, font: Font,
    private val minValue: Double,
    private val maxValue: Double,
    private val step: Double,
    private val read: () -> Double,
    private val write: (Double) -> Unit,
    description: String = "",
    private val unit: String = "",
) : TurnelerWidget(x, y, width, height, label, font, description) {
    private var dragging = false
    private val railWidth get() = minOf(164, width * 2 / 5).coerceAtLeast(1)
    private val railLeft get() = x + width - railWidth - 3
    private val railRight get() = x + width - 3

    private fun updateFromMouse(mouseX: Double) {
        val fraction = ((mouseX - railLeft) / railWidth).coerceIn(0.0, 1.0)
        val value = minValue + ((maxValue - minValue) * fraction / step).roundToInt() * step
        write(value.coerceIn(minValue, maxValue))
    }

    override fun activate() = Unit
    private fun formatted(value: Double): String {
        val decimals = when { step >= 1.0 -> 0; step >= 0.1 -> 1; step >= 0.01 -> 2; else -> 3 }
        return String.format(Locale.ROOT, "%.${decimals}f", value) + unit
    }
    override fun narratedValue() = formatted(read())
    override fun keyPressed(event: KeyEvent): Boolean {
        if (!active) return false
        val current = read().coerceIn(minValue, maxValue)
        when {
            event.isLeft() -> write((current - step).coerceAtLeast(minValue))
            event.isRight() -> write((current + step).coerceAtMost(maxValue))
            else -> return super.keyPressed(event)
        }
        return true
    }

    override fun mouseClicked(event: MouseButtonEvent, doubleClick: Boolean): Boolean {
        if (!active || event.button() != 0 || !isMouseOver(event.x(), event.y())) return false
        // Clicking the label focuses the setting without jumping its value to the minimum.
        if (event.x() >= railLeft - 6) {
            dragging = true
            updateFromMouse(event.x())
        }
        return true
    }

    override fun mouseDragged(event: MouseButtonEvent, dragX: Double, dragY: Double): Boolean {
        if (!active || !dragging) return false
        updateFromMouse(event.x())
        return true
    }

    override fun mouseReleased(event: MouseButtonEvent): Boolean {
        val handled = dragging
        dragging = false
        return handled
    }

    override fun extractWidgetRenderState(g: GuiGraphicsExtractor, mouseX: Int, mouseY: Int, delta: Float) {
        base(g)
        labels(g, railLeft - x - 14)
        val value = read().coerceIn(minValue, maxValue)
        textRight(g, formatted(value), x + width, y + 12, TurnelerTheme.MUTED)
        val railY = y + 36
        val fraction = (value - minValue) / (maxValue - minValue).coerceAtLeast(1e-9)
        val knob = (railLeft + railWidth * fraction).roundToInt()
        g.fill(railLeft, railY, railRight, railY + 2, TurnelerTheme.LINE)
        g.fill(railLeft, railY, knob, railY + 2, TurnelerTheme.ACCENT)
        TurnelerTheme.roundedFill(g, knob - 4, railY - 3, 8, 8, 4, TurnelerTheme.ACCENT)
    }
}
