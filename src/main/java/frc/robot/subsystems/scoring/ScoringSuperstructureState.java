package frc.robot.subsystems.scoring;

import frc.robot.subsystems.scoring.constants.ScoringConstants;

public enum ScoringSuperstructureState {
    HP_LOADING(
            0.5,
            0.75,
            0.5,
            true,
            false
    ),
    HANDOFF(
            0,
            0.5,
            0.5,
            true,
            false
    ),
    L1(
            0.5,
            0.5,
            -0.5,
            false,
            true
    ),
    L2(
            0.65,
            0.25,
            -0.5,
            false,
            true
    ),
    L3(
            0.8,
            0.25,
            -0.5,
            false,
            true
    ),
    L4(
            1,
            0,
            -0.5,
            false,
            true
    ),
    IDLE(
            0,
            1,
            0,
            false,
            true
    );

    private final double elevatorPosition;
    private final double wristPosition;
    public final double intakeSpeed;
    public final boolean intakeUntilSeen;
    public final boolean outtakeUntilSeen;
    public boolean stateFinished;

    ScoringSuperstructureState(
            double elevatorPosition,
            double wristPosition,
            double intakeSpeed,
            boolean intakeUntilSeen,
            boolean outtakeUntilSeen) {
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.intakeSpeed = intakeSpeed;
        this.intakeUntilSeen = intakeUntilSeen;
        this.outtakeUntilSeen = outtakeUntilSeen;
        stateFinished = false;
    }

    public double getElevatorAbsolutePosition() {
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * elevatorPosition;
    }

    public double getWristAbsolutePosition() {
        return ScoringConstants.WristConstants.DOWN_POSITION
                + ScoringConstants.WristConstants.POSITION_DIFF * wristPosition;
    }

    public void setStateFinished(boolean stateFinished) {
        this.stateFinished = stateFinished;
    }

    public boolean isStateFinished() {
        return stateFinished;
    }
}
