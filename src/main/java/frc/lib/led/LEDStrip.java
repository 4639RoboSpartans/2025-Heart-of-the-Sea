package frc.lib.led;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface LEDStrip extends Subsystem {
    void usePattern(LEDPattern pattern);

    void update();

    public static LEDStrip getInstance() {

    }

    @Override
    default void periodic() {
        update();
    }

    void resetToBlank();
}
