package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants;
import frc.robot.subsystems.scoring.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring.hopper.HopperSubsystem;

import static edu.wpi.first.units.Units.Inches;

@SuppressWarnings("rawtypes")
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

    private ScoringSuperstructureState withElevatorProportion(double percent) {
        this.elevatorPercent = percent;
        return this;
    }

    private ScoringSuperstructureState withWristProportion(double percent) {
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
            IDLE = new ScoringSuperstructureState()
            .withElevatorProportion(0.0)
            .withWristProportion(0)
            .withIntakeSpeed(0.0)
            .withIntakeUntilSeen(false)
            .withOuttakeUntilSeen(true)
            .withLastToMove(HopperSubsystem.class),

            HP_LOADING = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.HP_Proportion)
                    .withWristProportion(0)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(true)
                    .withOuttakeUntilSeen(false)
                    .withControl(Controls.Operator.HPLoadingTrigger)
                    .withStateAfter(IDLE),

            L1 = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.L1_Proportion)
                    .withWristProportion(1)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L1Trigger)
                    .withStateAfter(IDLE),

            L2 = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.L2_Proportion)
                    .withWristProportion(0.25)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L2Trigger)
                    .withStateAfter(IDLE),

            L3 = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.L3_Proportion)
                    .withWristProportion(0.25)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L3Trigger)
                    .withStateAfter(IDLE),

            L4 = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.L4_Proportion)
                    .withWristProportion(0.5)
                    .withIntakeSpeed(0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(true)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L4Trigger)
                    .withStateAfter(IDLE),

            L2_ALGAE = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.L2_ALGAE_Proportion)
                    .withWristProportion(1)
                    .withIntakeSpeed(-0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(false)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L2AlgaeTrigger)
                    .withStateAfter(IDLE),

            L3_ALGAE = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.L3_ALGAE_Proportion)
                    .withWristProportion(1)
                    .withIntakeSpeed(-0.5)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(false)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.L3AlgaeTrigger)
                    .withStateAfter(IDLE),

            BARGE_SCORING = new ScoringSuperstructureState()
                    .withElevatorProportion(ElevatorConstants.Barge_Proportion)
                    .withWristProportion(0.5)
                    .withIntakeSpeed(1)
                    .withIntakeUntilSeen(false)
                    .withOuttakeUntilSeen(false)
                    .withLastToMove(ElevatorSubsystem.class)
                    .withControl(Controls.Operator.BargeScoringTrigger)
                    .withStateAfter(IDLE);

    public double getElevatorAbsolutePosition() {
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
                + ScoringConstants.ElevatorConstants.POSITION_DIFF * elevatorPercent;
    }

    public double getWristAbsolutePosition() {
        return ScoringConstants.HopperConstants.EXTENDED_POSITION
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
                .plus(ScoringConstants.HopperConstants.IDLE_ROTATION);
    }

    public static double getWristSimPosition(Rotation2d rotation) {
        double position = (rotation.getDegrees() - ScoringConstants.HopperConstants.IDLE_ROTATION.getDegrees())
                / ScoringConstants.HopperConstants.MAX_ROTATION.getDegrees();
        return ScoringConstants.HopperConstants.EXTENDED_POSITION
                + ScoringConstants.HopperConstants.POSITION_DIFF * position;
    }

    public static Rotation2d getWristSimRotation(double position) {
        double rawRotation = (position - ScoringConstants.HopperConstants.EXTENDED_POSITION)
                / ScoringConstants.HopperConstants.POSITION_DIFF;
        return Rotation2d.fromDegrees(rawRotation * ScoringConstants.HopperConstants.MAX_ROTATION.getDegrees())
                .plus(ScoringConstants.HopperConstants.IDLE_ROTATION);
    }

    public ScoringSuperstructureState getStateAfter() {
        return stateAfter;
    }
}
