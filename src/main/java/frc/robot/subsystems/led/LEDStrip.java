package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.led.LEDPattern;
import frc.robot.subsystemManager.SubsystemInstantiator;

import java.util.Objects;
import java.util.function.Supplier;

public abstract class LEDStrip extends SubsystemBase {
    public abstract void setPattern(LEDPattern pattern);

    public abstract void doResetTime();

    public abstract void update();

    public static SubsystemInstantiator<LEDStrip> createInstance() {
        return new SubsystemInstantiator<>(
            () -> new PhysicalLEDStrip(
                9, 96
            ),
            DummyLEDStrip::new
        );
    }

    @Override
    public void periodic() {
        update();
    }

    public void resetToBlank() {
        setPattern(LEDPattern.BLANK);
    }

    public Command resetTime() {
        return runOnce(this::doResetTime).ignoringDisable(true);
    }

    public Command usePattern(LEDPattern pattern) {
        return usePattern(() -> pattern);
    }

    public Command usePattern(Supplier<LEDPattern> patternSupplier) {
        return run(() -> this.setPattern(patternSupplier.get())).finallyDo(() -> resetToBlank()).ignoringDisable(true);
    }
}
