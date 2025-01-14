package frc.robot.subsystems.scoring;

import frc.robot.subsystems.scoring.constants.ScoringConstants;

public enum ScoringSuperstructureState {
    HP_LOADING(
            0.5,
            0.75,
            0.5,
            true
    ),
    HANDOFF(
            0,
            0.5,
            0.5,
            true
    ),
    L1(
            0.5,
            0.5,
            -0.5,
            false
    ),
    L2(
            0.65,
            0.25,
            -0.5,
            false
    ),
    L3(
            0.8,
            0.25,
            -0.5,
            false
    ),
    L4(
            1,
            0,
            -0.5,
            false
    ),
    IDLE(
            0,
            1,
            0,
            false
    );

    private final double elevatorPosition;
    private final double wristPosition;
    public final double intakeSpeed;
    public final boolean intakeUntilSeen;

    ScoringSuperstructureState(
            double elevatorPosition,
            double wristPosition,
            double intakeSpeed,
            boolean intakeUntilSeen) {
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.intakeSpeed = intakeSpeed;
        this.intakeUntilSeen = intakeUntilSeen;
    }

    public double getElevatorAbsolutePosition() {
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * elevatorPosition;
    }

    public double getWristAbsolutePosition() {
        return ScoringConstants.WristConstants.DOWN_POSITION
                + ScoringConstants.WristConstants.POSITION_DIFF * wristPosition;
    }
}
