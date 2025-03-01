package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.annotation.PackagePrivate;
import frc.lib.led.LEDPattern;

import java.util.Objects;

public class PhysicalLEDStrip extends LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final int length;
    private double timeAtLastReset = Timer.getFPGATimestamp();

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
    public void setPattern(LEDPattern pattern) {
        currentPattern = pattern;
    }

    @Override
    public void update() {
        for (int i = 0; i < length; i++) {
            Color color = currentPattern.get(i, Timer.getFPGATimestamp() - timeAtLastReset);
            buffer.setLED(i, new Color8Bit(color));
        }
        led.setData(buffer);
    }

    @Override
    public void doResetTime() {
        timeAtLastReset = Timer.getFPGATimestamp();
    }
}