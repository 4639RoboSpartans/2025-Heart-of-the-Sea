package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring.hopper.HopperSubsystem;

import static edu.wpi.first.units.Units.Inches;

public class ScoringSuperstructureState {
    private double elevatorPercent;
    private double wristPercent;
    public double intakeSpeed;
    public boolean intakeUntilSeen;
    public boolean outtakeUntilSeen;
    public Class lastToMove;
    public Trigger control;
    public ScoringSuperstructureState stateAfter;

    private ScoringSuperstructureState() {
        this.elevatorPercent = 0;
        this.wristPercent = 0;
        this.intakeSpeed = 0;
        this.intakeUntilSeen = false;
        this.outtakeUntilSeen = false;
        this.lastToMove = null;
        this.control = null;
        this.stateAfter = this;
    }

    private ScoringSuperstructureState withElevatorPercent(double percent) {
        this.elevatorPercent = percent;
        return this;
    }

    private ScoringSuperstructureState withWristPercent(double percent) {
        this.wristPercent = percent;
        return this;
    }

    private ScoringSuperstructureState withIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
        return this;
    }

    private ScoringSuperstructureState withIntakeUntilSeen(boolean untilSeen) {
        this.intakeUntilSeen = untilSeen;
        return this;
    }

    private ScoringSuperstructureState withOuttakeUntilSeen(boolean untilSeen) {
        this.outtakeUntilSeen = untilSeen;
        return this;
    }

    private ScoringSuperstructureState withControl(Trigger control) {
        this.control = control;
        return this;
    }

    private ScoringSuperstructureState withLastToMove(Class lastToMove) {
        this.lastToMove = lastToMove;
        return this;
    }

    private ScoringSuperstructureState withStateAfter(ScoringSuperstructureState stateAfter) {
        this.stateAfter = stateAfter;
        return this;
    }

    public static final ScoringSuperstructureState
            IDLE =
                new ScoringSuperstructureState()
                        .withElevatorPercent(0.0)
                        .withWristPercent(1.0)
                        .withIntakeSpeed(0.0)
                        .withIntakeUntilSeen(false)
                        .withOuttakeUntilSeen(true)
                        .withLastToMove(HopperSubsystem.class),
            HP_LOADING =
                    new ScoringSuperstructureState()
                            .withElevatorPercent(0.5)
                            .withWristPercent(0.75)
                            .withIntakeSpeed(0.5)
                            .withIntakeUntilSeen(true)
                            .withOuttakeUntilSeen(false)
                            .withControl(Controls.Operator.HPLoadingTrigger),
            L1 = new ScoringSuperstructureState()
                    .withElevatorPercent(0.5)
                    .withWristPercent(0.5)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L1Trigger),
            L2 = new ScoringSuperstructureState()
                    .withElevatorPercent(0.65)
                    .withWristPercent(0.25)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L2Trigger),
            L3 = new ScoringSuperstructureState()
                    .withElevatorPercent(0.8)
                    .withWristPercent(0.25)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L3Trigger),
            L4 = new ScoringSuperstructureState()
                    .withElevatorPercent(1)
                    .withWristPercent(0)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L4Trigger),
            L2_ALGAE = new ScoringSuperstructureState()
                    .withElevatorPercent(0.65)
                    .withWristPercent(0.75)
                    .withIntakeSpeed(-0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(false)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L2AlgaeTrigger),
            L3_ALGAE = new ScoringSuperstructureState()
                    .withElevatorPercent(0.8)
                    .withWristPercent(0.75)
                    .withIntakeSpeed(-0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(false)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L3AlgaeTrigger),
            BARGE_SCORING = new ScoringSuperstructureState()
                    .withElevatorPercent(1)
                    .withWristPercent(0.5)
                    .withIntakeSpeed(1)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(false)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.BargeScoringTrigger);

    public double getElevatorAbsolutePosition() {
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * elevatorPercent;
    }

    public double getWristAbsolutePosition() {
        return ScoringConstants.HopperConstants.DOWN_POSITION
                + ScoringConstants.HopperConstants.POSITION_DIFF * wristPercent;
    }

    public Distance getElevatorLength() {
        return Inches.of(elevatorPercent
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

    public static Distance getElevatorSimDistance(double position) {
        double rawDist = (position - ScoringConstants.ElevatorConstants.DOWN_POSITION)
                / ScoringConstants.ElevatorConstants.POSITION_DIFF;
        return Inches.of(rawDist * ScoringConstants.ElevatorConstants.MAX_EXTENSION.in(Inches)
                + ScoringConstants.ElevatorConstants.STARTING_HEIGHT.in(Inches));
    }

    public Rotation2d getWristSimRotation() {
        return ScoringConstants.HopperConstants.MAX_ROTATION
                .times(wristPercent)
                .plus(ScoringConstants.HopperConstants.EXTENDED_ROTATION);
    }

    public static double getWristSimPosition(Rotation2d rotation) {
        double position = (rotation.getDegrees() - ScoringConstants.HopperConstants.EXTENDED_ROTATION.getDegrees())
                / ScoringConstants.HopperConstants.MAX_ROTATION.getDegrees();
        return ScoringConstants.HopperConstants.DOWN_POSITION
                + ScoringConstants.HopperConstants.POSITION_DIFF * position;
    }

    public static Rotation2d getWristSimRotation(double position) {
        double rawRotation = (position - ScoringConstants.HopperConstants.DOWN_POSITION)
                / ScoringConstants.HopperConstants.POSITION_DIFF;
        return Rotation2d.fromDegrees(rawRotation * ScoringConstants.HopperConstants.MAX_ROTATION.getDegrees())
                .plus(ScoringConstants.HopperConstants.EXTENDED_ROTATION);
    }

    public ScoringSuperstructureState getStateAfter() {
        return stateAfter;
    }
}
