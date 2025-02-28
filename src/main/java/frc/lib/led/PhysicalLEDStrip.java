package frc.lib.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.annotation.PackagePrivate;

import java.util.Objects;

public class PhysicalLEDStrip extends SubsystemBase implements LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final int length;

    private LEDPattern currentPattern = LEDPattern.BLANK;

    private static volatile PhysicalLEDStrip instance;

    @PackagePrivate
    static synchronized PhysicalLEDStrip getInstance() {
        return Objects.requireNonNullElseGet(instance, () -> instance = new PhysicalLEDStrip(9, 96));
    }

    private PhysicalLEDStrip(int port, int length) {
        this.length = length;

        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void usePattern(LEDPattern pattern) {
        currentPattern = pattern;
    }

    @Override
    public void update() {
        for (int i = 0; i < length; i++) {
            Color8Bit color = currentPattern.get(i, Timer.getFPGATimestamp());
            buffer.setLED(i, color);
        }
        led.setData(buffer);
    }

    @Override
    public void resetToBlank() {
        currentPattern = LEDPattern.BLANK;
    }
}