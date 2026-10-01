package adreno.turneler.client.mixin;

import adreno.turneler.client.TurnelerClient;
import net.minecraft.client.Minecraft;
import net.minecraft.world.entity.vehicle.boat.AbstractBoat;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(AbstractBoat.class)
public abstract class BoatControlMixin {
    @Shadow private float deltaRotation;

    @Inject(method = "controlBoat", at = @At("HEAD"))
    private void turneler$applyPursuit(CallbackInfo ci) {
        TurnelerClient.INSTANCE.getPilot().controlBoat(Minecraft.getInstance(), (AbstractBoat) (Object) this, deltaRotation);
    }
}
