package frc.lib.led;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SubsystemManager;

import java.util.Objects;

public interface LEDStrip extends Subsystem {
    void usePattern(LEDPattern pattern);

    void update();

    static LEDStrip getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);
        return (RobotBase.isReal() ? PhysicalLEDStrip.getInstance() : DummyLEDStrip.getInstance());
    }

    @Override
    default void periodic() {
        update();
    }

    void resetToBlank();
}
