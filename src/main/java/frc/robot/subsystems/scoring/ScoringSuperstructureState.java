package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants;

public class ScoringSuperstructureState {
    public double elevatorProportion; //proportion of the distance between lower and upper limit
    private double wristProportion; // proportion of the distance between lower and upper limit
    public double intakeSpeed; // speed of intake wheels
    public boolean intakeUntilGamePieceSeen; // whether to stop spinning intake wheels when game piece is seen
    public boolean outtakeUntilGamePieceNotSeen; // whether to stop spinning intake wheels after game piece isn't detected
    public boolean useTransitionState; // whether to move the endeffector to a transition state until the elevator is in position
    public Trigger control; // the trigger used to set this state
    public ScoringSuperstructureState stateAfter; // the state to set after this state finishes

    private ScoringSuperstructureState() {
        this.elevatorProportion = 0;
        this.wristProportion = 0;
        this.intakeSpeed = 0;
        this.intakeUntilGamePieceSeen = false;
        this.outtakeUntilGamePieceNotSeen = false;
        this.useTransitionState = true;
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

    private ScoringSuperstructureState withUseTransitionState(boolean useTransitionState) {
        this.useTransitionState = useTransitionState;
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
            .withUseTransitionState(false);
    }

    public static final ScoringSuperstructureState
        IDLE = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.IDLE_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_IDLE_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_IDLE_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withUseTransitionState(true),

    HP_LOADING = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.HP_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_HP_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_HP_Speed)
        .withIntakeUntilSeen(true)
        .withOuttakeUntilNotSeen(false)
        .withUseTransitionState(false)
        .withControl(Controls.Operator.HPLoadingTrigger)
        .withStateAfter(IDLE),

    PROCESSOR_SCORING = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.Processor_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_Processor_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_Processor_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false)
        .withUseTransitionState(true)
        .withControl(Controls.Driver.ProcessorTrigger)
        .withStateAfter(IDLE),

    L1 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L1_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_L1_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_L1_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withUseTransitionState(true)
        .withControl(Controls.Operator.L1Trigger)
        .withStateAfter(IDLE),

    L2 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L2_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_L2_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_L2_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withUseTransitionState(true)
        .withControl(Controls.Operator.L2Trigger)
        .withStateAfter(IDLE),

    L3 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L3_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_L3_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_L3_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withUseTransitionState(true)
        .withControl(Controls.Operator.L3Trigger)
        .withStateAfter(IDLE),

    L4 = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L4_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_L4_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_L4_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(true)
        .withUseTransitionState(true)
        .withControl(Controls.Operator.L4Trigger)
        .withStateAfter(IDLE),

    L2_ALGAE = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L2_ALGAE_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_L2_ALGAE_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_L2_ALGAE_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false)
        .withUseTransitionState(true)
        .withControl(Controls.Driver.L2AlgaeTrigger)
        .withStateAfter(IDLE),

    L3_ALGAE = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.L3_ALGAE_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_L3_ALGAE_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_L3_ALGAE_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false)
        .withUseTransitionState(true)
        .withControl(Controls.Driver.L3AlgaeTrigger)
        .withStateAfter(IDLE),

    BARGE_SCORING = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.Barge_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_Barge_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_Barge_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false)
        .withUseTransitionState(true)
        .withControl(Controls.Driver.BargeScoringTrigger)
        .withStateAfter(IDLE),

    TRANSITION_STATE = new ScoringSuperstructureState()
        .withElevatorProportion(ElevatorConstants.Proportions.Transition_Proportion)
        .withWristProportion(EndEffectorConstants.Proportions.Wrist_Transition_Proportion)
        .withIntakeSpeed(EndEffectorConstants.Intake_Transition_Speed)
        .withIntakeUntilSeen(false)
        .withOuttakeUntilNotSeen(false);

    public double getElevatorAbsolutePosition() {
        return ElevatorConstants.ProportionToPosition.convert(elevatorProportion);
    }

    public double getWristAbsolutePosition() {
        return EndEffectorConstants.ProportionToPosition.convert(wristProportion);
    }

    public Distance getElevatorHeight() {
        return ElevatorConstants.ProportionToHeight.convert(elevatorProportion);
    }

    public Rotation2d getRotation() {
        return EndEffectorConstants.ProportionToRotation.convert(wristProportion);
    }

    public static double getWristSimPosition(Rotation2d rotation) {
        return EndEffectorConstants.PositionToRotation.convertBackwards(rotation);
    }

    public static Rotation2d getRotation(double position) {
        return EndEffectorConstants.PositionToRotation.convert(position);
    }

    public ScoringSuperstructureState getStateAfter() {
        return stateAfter;
    }
}
