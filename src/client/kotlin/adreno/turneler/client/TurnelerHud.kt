package adreno.turneler.client

import net.fabricmc.fabric.api.client.rendering.v1.hud.HudElement
import net.minecraft.client.DeltaTracker
import net.minecraft.client.Minecraft
import net.minecraft.client.gui.Font
import net.minecraft.client.gui.GuiGraphicsExtractor

/** A quiet overlay shared with the settings screen's placement preview. */
object TurnelerHud : HudElement {
    private const val PADDING = 10
    private const val LINE_HEIGHT = 14
    private const val MAX_LINE_WIDTH = 240

    override fun extractRenderState(graphics: GuiGraphicsExtractor, deltaTracker: DeltaTracker) {
        val client = Minecraft.getInstance()
        val config = TurnelerClient.config
        if (config.hudHidden || client.player == null || client.gui.screen() is TurnelerConfigScreen) return
        draw(graphics, client.font, config.hudX, config.hudY, TurnelerClient.pilot.hudLines)
    }

    fun measure(font: Font, lines: List<String>): IntArray {
        if (lines.isEmpty()) return intArrayOf(0, 0)
        val shown = lines.map { font.plainSubstrByWidth(it, MAX_LINE_WIDTH) }
        return intArrayOf(shown.maxOf { font.width(it) } + 2 * PADDING, lines.size * LINE_HEIGHT + 2 * PADDING + 4)
    }

    fun placed(x: Int, y: Int, size: IntArray): IntArray {
        val window = Minecraft.getInstance().window
        return intArrayOf(
            x.coerceIn(0, maxOf(0, window.guiScaledWidth - size[0])),
            y.coerceIn(0, maxOf(0, window.guiScaledHeight - size[1])),
        )
    }

    fun contains(font: Font, x: Int, y: Int, lines: List<String>, mouseX: Double, mouseY: Double): Boolean {
        val size = measure(font, lines)
        val corner = placed(x, y, size)
        return size[0] > 0 && mouseX >= corner[0] && mouseX < corner[0] + size[0] &&
            mouseY >= corner[1] && mouseY < corner[1] + size[1]
    }

    fun draw(g: GuiGraphicsExtractor, font: Font, x: Int, y: Int, lines: List<String>) {
        if (lines.isEmpty()) return
        val size = measure(font, lines)
        val corner = placed(x, y, size)
        val left = corner[0]
        val top = corner[1]
        TurnelerTheme.roundedFill(g, left, top, size[0], size[1], 8, 0xEAF8F8FA.toInt())
        g.fill(left + PADDING, top + 23, left + size[0] - PADDING, top + 24, TurnelerTheme.LINE)
        lines.forEachIndexed { index, line ->
            val yOffset = if (index == 0) 0 else 4
            g.text(font, font.plainSubstrByWidth(line, MAX_LINE_WIDTH), left + PADDING,
                top + PADDING + index * LINE_HEIGHT + yOffset,
                if (index == 0) TurnelerTheme.TEXT else TurnelerTheme.MUTED, false)
        }
    }
}
