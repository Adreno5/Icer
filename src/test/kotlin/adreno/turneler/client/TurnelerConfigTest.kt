package adreno.turneler.client

import adreno.turneler.navigation.IcerParameter
import com.google.gson.GsonBuilder
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class TurnelerConfigTest {
    @Test fun legacyModesDisappearWhileUserSettingsSurvive() {
        val config = TurnelerConfig.fromJson("""{"mode":"JEV","enabled":false,"icerHorizon":60,
            "hudX":36,"showPrediction":false,"jevRays":32}""")
        assertFalse(config.enabled)
        assertFalse(config.showPrediction)
        assertEquals(60, config.icerHorizon)
        assertEquals(36, config.hudX)
        assertEquals(IcerParameter.entries.size, config.parameters.size)
        val saved = GsonBuilder().create().toJson(config)
        assertFalse(saved.contains("mode"))
        assertFalse(saved.contains("jevRays"))
        assertEquals(config, TurnelerConfig.fromJson(saved))
    }

    @Test fun malformedIndividualParametersFallBackWithoutLosingOtherSettings() {
        val config = TurnelerConfig.fromJson("""{"enabled":false,"parameters":{
            "lateral_damping":"bad","beam":99999,"fine_step":3,"coarse_step":0.75,
            "rate_deadzone":1.2,"unknown":5}}""")
        assertFalse(config.enabled)
        assertEquals(3.0, config.parameters["lateral_damping"])
        assertEquals(32.0, config.parameters["beam"])
        assertEquals(0.75, config.parameters["fine_step"])
        assertEquals(1.2, config.parameters["rate_deadzone"])
        assertFalse(config.parameters.containsKey("unknown"))
    }

    @Test fun obsoleteControllerParametersDisappearAndNewTuningRoundTrips() {
        val config = TurnelerConfig.fromJson("""{"enabled":false,"icerHorizon":60,"parameters":{
            "narrow_brake":0.95,"narrow_start":0.3,"narrow_band":0.25,"yaw_deadzone":0.5,
            "launch_speed":0.08,"launch_angle":45,"launch_thrust":0.01,
            "shortcut_weight":0.4,"reference_rate_limit":6.25,"free_feedback_width":2.5,
            "reverse_slow_period":5.1,"reverse_fast_period":7.4,"coast_brake_speed":"bad"}}""")
        assertFalse(config.enabled)
        assertEquals(60, config.icerHorizon)
        for (key in listOf("narrow_brake", "narrow_start", "narrow_band", "yaw_deadzone",
            "launch_speed", "launch_angle", "launch_thrust")) assertFalse(config.parameters.containsKey(key))
        assertEquals(0.4, config.parameters["shortcut_weight"], "existing user tuning survives")
        assertEquals(6.25, config.parameters["reference_rate_limit"])
        assertEquals(2.5, config.parameters["free_feedback_width"])
        assertEquals(7.0, config.parameters["reverse_fast_period"])
        assertEquals(7.0, config.parameters["reverse_slow_period"], "low-speed period cannot be shorter")
        assertEquals(1.5, config.parameters["coast_brake_speed"])
        assertEquals(config, TurnelerConfig.fromJson(GsonBuilder().create().toJson(config)))
    }

    @Test fun lastSectionAndIndependentScrollPositionsSurviveAConfigRoundTrip() {
        val config = TurnelerConfig()
        val view = config.settingsView
        view.remember("tuning", null, 104)
        view.remember("tuning", "弯道制动", 312)
        view.remember("display", null, 156)
        // Visiting display retains the last tuning category, while each page keeps its own scroll.
        assertEquals("弯道制动", view.group)
        view.remember("tuning", "弯道制动", view.scrollFor("tuning", "弯道制动"))
        val restored = TurnelerConfig.fromJson(GsonBuilder().create().toJson(config)).settingsView
        assertEquals("tuning", restored.page)
        assertEquals("弯道制动", restored.group)
        assertEquals(312, restored.scrollFor("tuning", "弯道制动"))
        assertEquals(104, restored.scrollFor("tuning", null))
        assertEquals(156, restored.scrollFor("display", null))
    }

    @Test fun obsoleteOrMalformedViewHistoryDoesNotResetDrivingSettings() {
        val config = TurnelerConfig.fromJson("""{"enabled":false,"icerHorizon":60,"settingsView":{
            "page":"removed-mode","group":"missing-category","scrollPositions":{
                "display":-50,"tuning":"bad","tuning/弯道制动":999999,"obsolete":42}}}""")
        assertFalse(config.enabled)
        assertEquals(60, config.icerHorizon)
        assertEquals("drive", config.settingsView.page)
        assertNull(config.settingsView.group)
        assertEquals(0, config.settingsView.scrollFor("display", null))
        assertEquals(16384, config.settingsView.scrollFor("tuning", "弯道制动"))
        assertFalse(config.settingsView.scrollPositions.containsKey("obsolete"))
        assertEquals(SettingsViewState(), TurnelerConfig.fromJson("{}").settingsView)
    }
}
