package adreno.turneler.client

import adreno.turneler.navigation.IcerParameter
import com.google.gson.JsonElement

/** Navigation history only; restoring driving defaults does not clear a user's place in settings. */
data class SettingsViewState(
    var page: String = "drive",
    var group: String? = null,
    var scrollPositions: Map<String, Int> = emptyMap(),
) {
    fun sanitize(): SettingsViewState {
        if (page !in PAGES) page = "drive"
        if (group !in GROUPS) group = null
        scrollPositions = scrollPositions.filterKeys { it in KEYS }.mapValues { it.value.coerceIn(0, MAX_SCROLL) }
        return this
    }

    fun scrollFor(page: String, group: String?) = scrollPositions[key(page, group)] ?: 0

    fun remember(page: String, group: String?, scroll: Int) {
        this.page = page
        if (page == "tuning") this.group = group
        scrollPositions = scrollPositions + (key(page, group) to scroll.coerceIn(0, MAX_SCROLL))
    }

    companion object {
        private val PAGES = setOf("drive", "tuning", "display")
        private val GROUPS = IcerParameter.entries.map { it.group }.toSet()
        private val KEYS = PAGES + GROUPS.map { "tuning/$it" }
        private const val MAX_SCROLL = 16384

        private fun key(page: String, group: String?) = if (page == "tuning" && group != null) "tuning/$group" else page

        fun fromJson(raw: JsonElement?): SettingsViewState {
            val root = runCatching { raw?.asJsonObject }.getOrNull()
            val page = runCatching { root?.get("page")?.asString }.getOrNull() ?: "drive"
            val group = runCatching { root?.get("group")?.asString }.getOrNull()
            val positions = runCatching { root?.getAsJsonObject("scrollPositions") }.getOrNull()
            val scrolls = KEYS.mapNotNull { key ->
                val value = runCatching { positions?.get(key)?.asDouble }.getOrNull()
                if (value == null || !value.isFinite()) null else key to value.coerceIn(0.0, MAX_SCROLL.toDouble()).toInt()
            }.toMap()
            return SettingsViewState(page, group, scrolls).sanitize()
        }
    }
}
