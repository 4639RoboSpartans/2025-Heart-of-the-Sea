package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Voltage;

public class DummyElevatorSubsystem extends AbstractElevatorSubsystem {

    public DummyElevatorSubsystem() {
        super();
    }

    @Override
    public double getCurrentExtensionFraction() {
        return getTargetProportion();
    }

    @Override
    public void setRawMotorVoltage(Voltage voltage) {}
}
