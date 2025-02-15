package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public class DummyElevatorSubsystem extends ElevatorSubsystem {

    public DummyElevatorSubsystem() {
        super();
    }

    @Override
    public double getCurrentProportion() {
        return state.elevatorProportion;
    }

    @Override
    public boolean isAtTarget() {
        return true;
    }

    @Override
    public boolean isElevatorStateFinished() {
        return true;
    }

    @Override
    public void updateElevatorState(ScoringSuperstructureState state) {
        this.state = state;
    }

    @Override
    public void runElevator() {}

    @Override
    public void setRawMotorVoltage(Voltage voltage) {}
}
