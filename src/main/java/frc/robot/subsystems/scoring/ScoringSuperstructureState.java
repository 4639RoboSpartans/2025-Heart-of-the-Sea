package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.HopperConstants;
import frc.robot.subsystems.scoring.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring.hopper.HopperSubsystem;

import static edu.wpi.first.units.Units.Inches;

@SuppressWarnings("rawtypes")
public class ScoringSuperstructureState {
    private double elevatorProportion; //proportion of the distance between lower and upper limit
    private double wristProportion; // proportion of the distance between lower and upper limit
    public double intakeSpeed; // speed of intake wheels
    public boolean intakeUntilGamePieceSeen; // whether to stop spinning intake wheels when game piece is seen
    public boolean outtakeUntilGamePieceNotSeen; // whether to stop spinning intake wheels after game piece isn't detected
    public Class lastToMove; // the last "sub-subsystem" to move when moving to any other state
    public Trigger control; // the trigger used to set this state
    public ScoringSuperstructureState stateAfter; // the state to set after this state finishes

    private ScoringSuperstructureState() {
        this.elevatorProportion = 0;
        this.wristProportion = 0;
        this.intakeSpeed = 0;
        this.intakeUntilGamePieceSeen = false;
        this.outtakeUntilGamePieceNotSeen = false;
        this.lastToMove = null;
        this.control = new Trigger(() -> true);
        this.stateAfter = this;
    }

    private ScoringSuperstructureState withElevatorProportion(double percent) {
        this.elevatorProportion = percent;
        return this;
    }

    private ScoringSuperstructureState withWristProportion(double percent) {
        this.wristProportion = percent;
        return this;
    }

    private ScoringSuperstructureState withIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
        return this;
    }

    private ScoringSuperstructureState withIntakeUntilSeen(boolean untilSeen) {
        this.intakeUntilGamePieceSeen = untilSeen;
        return this;
    }

    private ScoringSuperstructureState withOuttakeUntilNotSeen(boolean untilSeen) {
        this.outtakeUntilGamePieceNotSeen = untilSeen;
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

    public static ScoringSuperstructureState HOLD(double elevatorProportion, double wristProportion) {
        return new ScoringSuperstructureState()
            .withElevatorProportion(elevatorProportion)
            .withWristProportion(wristProportion)
            .withIntakeSpeed(0)
            .withIntakeUntilSeen(false)
            .withOuttakeUntilNotSeen(false)
            .withLastToMove(null);
    }

    public static final ScoringSuperstructureState
        IDLE = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.IDLE_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_IDLE_Proportion)
        .withIntakeSpeed(0.0)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withLastToMove(HopperSubsystem.class),

    HP_LOADING = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.HP_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_HP_Proportion)
        .withIntakeSpeed(0.5)
        .withIntakeUntilSeen(true)
        .withOuttakeUntilNotSeen(false)
        .withControl(Controls.Operator.HPLoadingTrigger)
        .withStateAfter(IDLE),

    L1 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L1_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_L1_Proportion)
        .withIntakeSpeed(0.5)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withLastToMove(ElevatorSubsystem.class)
        .withControl(Controls.Operator.L1Trigger)
        .withStateAfter(IDLE),

    L2 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L2_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_L2_Proportion)
        .withIntakeSpeed(0.5)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withLastToMove(ElevatorSubsystem.class)
        .withControl(Controls.Operator.L2Trigger)
        .withStateAfter(IDLE),

    L3 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L3_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_L3_Proportion)
        .withIntakeSpeed(0.5)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withLastToMove(ElevatorSubsystem.class)
        .withControl(Controls.Operator.L3Trigger)
        .withStateAfter(IDLE),

    L4 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L4_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_L4_Proportion)
        .withIntakeSpeed(0.5)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withLastToMove(ElevatorSubsystem.class)
        .withControl(Controls.Operator.L4Trigger)
        .withStateAfter(IDLE),

    L2_ALGAE = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L2_ALGAE_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_L2_ALGAE_Proportion)
        .withIntakeSpeed(-0.5)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false)
        .withLastToMove(ElevatorSubsystem.class)
        .withControl(Controls.Operator.L2AlgaeTrigger)
        .withStateAfter(IDLE),

    L3_ALGAE = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L3_ALGAE_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_L3_ALGAE_Proportion)
        .withIntakeSpeed(-0.5)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false)
        .withLastToMove(ElevatorSubsystem.class)
        .withControl(Controls.Operator.L3AlgaeTrigger)
        .withStateAfter(IDLE),

    BARGE_SCORING = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.Barge_Proportion)
        .withWristProportion(HopperConstants.Proportions.Wrist_Barge_Proportion)
        .withIntakeSpeed(1)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false)
        .withLastToMove(ElevatorSubsystem.class)
        .withControl(Controls.Operator.BargeScoringTrigger)
        .withStateAfter(IDLE),

    TRANSITION_STATE = new ScoringSuperstructureState()
        .withElevatorProportion(0)
        .withWristProportion(0.5)
        .withIntakeSpeed(0)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false);

    public double getElevatorAbsolutePosition() {
        return ScoringConstants.ElevatorConstants.DOWN_POSITION
            + ScoringConstants.ElevatorConstants.POSITION_DIFF * elevatorProportion;
    }

    public double getWristAbsolutePosition() {
        return ScoringConstants.HopperConstants.EXTENDED_POSITION
            + ScoringConstants.HopperConstants.POSITION_DIFF * wristProportion;
    }

    public Distance getElevatorLength() {
        return Inches.of(elevatorProportion
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
            .times(wristProportion)
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
