package frc.robot.subsystems.led;

import frc.lib.annotation.PackagePrivate;
import frc.lib.led.LEDPattern;

import java.util.Objects;

public class DummyLEDStrip extends LEDStrip {

    private static volatile LEDStrip instance;

    @PackagePrivate
    static synchronized LEDStrip getInstance() {
        return Objects.requireNonNullElseGet(instance, () -> instance = new DummyLEDStrip());
    }

    @Override
    public void setPattern(LEDPattern pattern) {

    }

    @Override
    public void doResetTime() {

    }

    @Override
    public void update() {

    }
}
