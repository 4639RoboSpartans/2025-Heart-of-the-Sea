package frc.robot.subsystems.scoring;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import static edu.wpi.first.units.Units.Inches;


public enum ScoringSuperstructureState {
    HP_LOADING(
            0.5,
            0.75,
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
    ),
    DUNK_L2(
            L2,
            -0.05,
            0
    ),
    DUNK_L3(
            L3,
            -0.05,
            0
    ),
    DUNK_L4(
            L4,
            -0.1,
            0
    ),
    L2_ALGAE(
            0.65,
            0.75,
            -0.5,
            false,
            true
    ),
    L3_ALGAE(
            0.8,
            0.75,
            -0.5,
            false,
            true
    ),
    BARGE_SCORING(
            1,
            0.5,
            1,
            false,
            false
    );

    private final double elevatorPosition;
    private final double wristPosition;
    public final double intakeSpeed;
    public final boolean intakeUntilSeen;
    public final boolean outtakeUntilSeen;

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
    }

    ScoringSuperstructureState(
            ScoringSuperstructureState prevState,
            double elevatorPositionDelta,
            double wristPositionDelta
    ) {
        elevatorPosition = prevState.elevatorPosition + elevatorPositionDelta;
        wristPosition = prevState.wristPosition + wristPositionDelta;
        intakeSpeed = prevState.intakeSpeed;
        intakeUntilSeen = prevState.intakeUntilSeen;
        outtakeUntilSeen = prevState.outtakeUntilSeen;
    }

    public double getElevatorAbsolutePosition() {
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * elevatorPosition;
    }

    public double getWristAbsolutePosition() {
        return ScoringConstants.WristConstants.DOWN_POSITION
                + ScoringConstants.WristConstants.POSITION_DIFF * wristPosition;
    }

    public Distance getElevatorSimPosition() {
        return Inches.of(elevatorPosition * 84 + 36);
    }

    public static double getElevatorSimPosition(Distance distance) {
        double position = (distance.in(Inches) - 36.0) / 84;
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * position;
    }

    public ScoringSuperstructureState getStateAfter() {
        return switch (this) {
            case HP_LOADING, L1, DUNK_L2, DUNK_L3, DUNK_L4, L2_ALGAE, L3_ALGAE, BARGE_SCORING -> IDLE;
            case L2 -> DUNK_L2;
            case L3 -> DUNK_L3;
            case L4 -> DUNK_L4;
            default -> this;
        };
    }
}
