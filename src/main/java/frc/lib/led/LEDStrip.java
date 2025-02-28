package frc.lib.led;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SubsystemManager;

public interface LEDStrip extends Subsystem {
    void usePattern(LEDPattern pattern);

    void update();

    public static LEDStrip getInstance() {
        return (RobotBase.isReal() ? PhysicalLEDStrip.getInstance() : DummyLEDStrip.getInstance());
    }

    @Override
    default void periodic() {
        update();
    }

    void resetToBlank();
}
