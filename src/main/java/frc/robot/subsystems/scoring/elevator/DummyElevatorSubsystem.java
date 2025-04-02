package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ElevatorSetpoints;

public class DummyElevatorSubsystem extends AbstractElevatorSubsystem {

    public DummyElevatorSubsystem() {
        super();
    }

    @Override
    public ElevatorPosition getCurrentPosition() {
        return getTargetPosition();
    }

    @Override
    public void setRawMotorVoltage(Voltage voltage) {}

    @Override
    public boolean isPhysicallyStopped() {
        return !ElevatorSetpoints.AllowedRange.contains(getCurrentPosition());
    }

    @Override
    public void resetCurrentPositionTo(ElevatorPosition position) {}
}
