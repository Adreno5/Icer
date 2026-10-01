package adreno.turneler.client

import adreno.turneler.navigation.IcerParameter
import adreno.turneler.navigation.IcerSettings
import com.google.gson.GsonBuilder
import com.google.gson.JsonParser
import net.fabricmc.loader.api.FabricLoader
import java.nio.file.Files
import java.nio.file.Path

data class TurnelerConfig(
    var enabled: Boolean = true,
    var showPrediction: Boolean = true,
    var alwaysOnTop: Boolean = true,
    var icerHorizon: Int = 48,
    var hudHidden: Boolean = false,
    var hudX: Int = 6,
    var hudY: Int = 6,
    var parameters: Map<String, Double> = emptyMap(),
    var settingsView: SettingsViewState = SettingsViewState(),
) {
    fun sanitize(): TurnelerConfig {
        icerHorizon = icerHorizon.coerceIn(24, 72)
        hudX = hudX.coerceIn(0, 4096)
        hudY = hudY.coerceIn(0, 4096)
        parameters = IcerSettings(parameters).toMap()
        settingsView.sanitize()
        return this
    }

    fun save() {
        sanitize()
        val path = configPath()
        Files.createDirectories(path.parent)
        Files.writeString(path, GSON.toJson(this))
    }

    companion object {
        private val GSON = GsonBuilder().setPrettyPrinting().create()

        private fun configPath(): Path = FabricLoader.getInstance().configDir.resolve("turneler.json")

        /** Read legacy settings too; obsolete mode fields are ignored and removed on the next save. */
        fun fromJson(json: String): TurnelerConfig {
            val root = JsonParser.parseString(json).asJsonObject
            val raw = root.remove("parameters")
            val view = root.remove("settingsView")
            val config = GSON.fromJson(root, TurnelerConfig::class.java) ?: TurnelerConfig()
            config.parameters = IcerParameter.entries.associate { parameter ->
                val value = runCatching { raw?.asJsonObject?.get(parameter.key)?.asDouble }.getOrNull()
                parameter.key to parameter.sanitize(value)
            }
            config.settingsView = SettingsViewState.fromJson(view)
            return config.sanitize()
        }

        fun load(): TurnelerConfig {
            val path = configPath()
            if (!Files.exists(path)) return TurnelerConfig().also { it.save() }
            return (
                runCatching { fromJson(Files.readString(path)) }
                    .getOrNull() ?: TurnelerConfig()
            ).sanitize()
        }
    }
}
