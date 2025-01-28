package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import static edu.wpi.first.units.Units.Inches;


public enum ScoringSuperstructureState {
    IDLE(
            0,
            1,
            0,
            false,
            true,
            ScoringSubsystem.ELEVATOR,
            null
    ),
    HP_LOADING(
            0.5,
            0.75,
            0.5,
            true,
            false,
            ScoringSubsystem.BOTH,
            Controls.Operator.HPLoadingTrigger
    ),
    L1(
            0.5,
            0.5,
            -0.5,
            false,
            true,
            ScoringSubsystem.HOPPER,
            Controls.Operator.L1Trigger
    ),
    L2(
            0.65,
            0.25,
            -0.5,
            false,
            true,
            ScoringSubsystem.HOPPER,
            Controls.Operator.L2Trigger
    ),
    L3(
            0.8,
            0.25,
            -0.5,
            false,
            true,
            ScoringSubsystem.HOPPER,
            Controls.Operator.L3Trigger
    ),
    L4(
            1,
            0,
            -0.5,
            false,
            true,
            ScoringSubsystem.HOPPER,
            Controls.Operator.L4Trigger
    ),
    L2_ALGAE(
            0.65,
            0.75,
            -0.5,
            false,
            true,
            ScoringSubsystem.HOPPER,
            Controls.Operator.L2AlgaeTrigger
    ),
    L3_ALGAE(
            0.8,
            0.75,
            -0.5,
            false,
            true,
            ScoringSubsystem.HOPPER,
            Controls.Operator.L3AlgaeTrigger
    ),
    BARGE_SCORING(
            1,
            0.5,
            1,
            false,
            false,
            ScoringSubsystem.HOPPER,
            Controls.Operator.BargeScoringTrigger
    );

    private final double elevatorPosition;
    private final double wristPosition;
    public final double intakeSpeed;
    public final boolean intakeUntilSeen;
    public final boolean outtakeUntilSeen;
    public final ScoringSubsystem firstToMove;
    public final Trigger control;

    ScoringSuperstructureState(
            double elevatorPosition,
            double wristPosition,
            double intakeSpeed,
            boolean intakeUntilSeen,
            boolean outtakeUntilSeen,
            ScoringSubsystem firstToMove,
            Trigger control) {
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.intakeSpeed = intakeSpeed;
        this.intakeUntilSeen = intakeUntilSeen;
        this.outtakeUntilSeen = outtakeUntilSeen;
        this.firstToMove = firstToMove;
        this.control = control;
    }

    public double getElevatorAbsolutePosition() {
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * elevatorPosition;
    }

    public double getWristAbsolutePosition() {
        return ScoringConstants.HopperConstants.DOWN_POSITION
                + ScoringConstants.HopperConstants.POSITION_DIFF * wristPosition;
    }

    public Distance getElevatorSimLength() {
        return Inches.of(elevatorPosition
                * ScoringConstants.ElevatorConstants.MAX_EXTENSION.in(Inches)
                + ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Inches)
        );
    }

    public static double getElevatorSimPosition(Distance distance) {
        double position = (distance.in(Inches) - ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Inches))
                / ScoringConstants.ElevatorConstants.MAX_EXTENSION.in(Inches);
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * position;
    }

    public Rotation2d getWristSimRotation() {
        return ScoringConstants.HopperConstants.MAX_ROTATION
                .times(wristPosition)
                .plus(ScoringConstants.HopperConstants.EXTENDED_ROTATION);
    }

    public static double getWristSimPosition(Rotation2d rotation) {
        double position = (rotation.getDegrees() - ScoringConstants.HopperConstants.EXTENDED_ROTATION.getDegrees())
                / ScoringConstants.HopperConstants.MAX_ROTATION.getDegrees();
        return ScoringConstants.HopperConstants.DOWN_POSITION
                + ScoringConstants.HopperConstants.POSITION_DIFF * position;
    }

    public ScoringSuperstructureState getStateAfter() {
        return switch (this) {
            case HP_LOADING, BARGE_SCORING -> IDLE;
            default -> this;
        };
    }

    public enum ScoringSubsystem {
        HOPPER,
        ELEVATOR,
        BOTH
    }
}
