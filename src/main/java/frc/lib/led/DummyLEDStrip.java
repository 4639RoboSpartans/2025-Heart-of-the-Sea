package frc.lib.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Objects;

public class DummyLEDStrip extends SubsystemBase implements LEDStrip {

    private static volatile LEDStrip instance;

    public static synchronized LEDStrip getInstance() {
        return Objects.requireNonNullElseGet(instance, () -> instance = new DummyLEDStrip());
    }

    @Override
    public void usePattern(LEDPattern pattern) {

    }

    @Override
    public void update() {

    }

    @Override
    public void resetToBlank() {

    }
}
