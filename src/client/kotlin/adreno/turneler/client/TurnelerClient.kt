package adreno.turneler.client

import adreno.turneler.Turneler
import com.mojang.blaze3d.platform.InputConstants
import net.fabricmc.api.ClientModInitializer
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents
import net.fabricmc.fabric.api.client.keymapping.v1.KeyMappingHelper
import net.fabricmc.fabric.api.client.rendering.v1.hud.HudElementRegistry
import net.fabricmc.fabric.api.client.rendering.v1.level.LevelRenderEvents
import net.minecraft.client.KeyMapping
import org.lwjgl.glfw.GLFW

object TurnelerClient : ClientModInitializer {
    lateinit var config: TurnelerConfig
        private set

    lateinit var pilot: BoatPilot
        private set

    private lateinit var configKey: KeyMapping

    override fun onInitializeClient() {
        config = TurnelerConfig.load()
        pilot = BoatPilot(config)
        configKey =
            KeyMappingHelper.registerKeyMapping(
                KeyMapping(
                    "key.turneler.config",
                    InputConstants.Type.KEYSYM,
                    GLFW.GLFW_KEY_RIGHT_SHIFT,
                    KeyMapping.Category.MISC,
                ),
            )

        ClientTickEvents.END_CLIENT_TICK.register { client ->
            while (configKey.consumeClick()) {
                if (client.gui.screen() == null) client.gui.setScreen(TurnelerConfigScreen(client, config))
            }
            pilot.tick(client)
        }

        LevelRenderEvents.BEFORE_GIZMOS.register {
            pilot.renderGizmos()
        }

        HudElementRegistry.addLast(Turneler.id("hud"), TurnelerHud)
    }
}
