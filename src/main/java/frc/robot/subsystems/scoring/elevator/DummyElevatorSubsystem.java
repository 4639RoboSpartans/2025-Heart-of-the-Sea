package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import static edu.wpi.first.units.Units.Inches;

public class DummyElevatorSubsystem extends ElevatorSubsystem {
    ScoringSuperstructureState scoringState;
    public DummyElevatorSubsystem() {
        super();
        scoringState = ScoringSuperstructureState.IDLE;
    }

    @Override
    public double getCurrentPosition() {
        return scoringState.getElevatorAbsolutePosition();
    }

    @Override
    public Distance getCurrentLength() {
        return Distance.ofBaseUnits(ScoringConstants.ElevatorConstants.ProportionToPosition.convert(getCurrentPosition()), Inches);
    }

    @Override
    public double getTargetPosition() {
        return getCurrentPosition();
    }

    @Override
    public Distance getTargetLength() {
        return getCurrentLength();
    }

    @Override
    public boolean isElevatorAtPosition() {
        return true;
    }

    @Override
    public boolean isElevatorStateFinished() {
        return true;
    }

    @Override
    public void setElevatorState(ScoringSuperstructureState state) {
        scoringState = state;
    }

    @Override
    public void runElevator() {
        //five
    }

    @Override
    public void setElevatorMotorVoltsSysID(Voltage voltage) {
        //big
        //booms
    }
}
