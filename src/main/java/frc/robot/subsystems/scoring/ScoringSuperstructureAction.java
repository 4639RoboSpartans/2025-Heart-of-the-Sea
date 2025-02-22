package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.IntakeSpeeds;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.WristSetpoints;

public class ScoringSuperstructureAction {
    public double targetElevatorExtensionFraction = 0;
    public double targetWristRotationFraction = 0;
    public double intakeSpeed = 0;
    public boolean endOnGamePieceSeen = false;
    public boolean endOnGamePieceNotSeen = false;
    public boolean requiresWristTransition = false;
    /** The trigger used to activate the state */
    public Trigger trigger = new Trigger(() -> true);
    /**
     * The state to use after this state has finished. Usually, this will
     * be the idle state
     */
    public ScoringSuperstructureAction nextAction = this;

    private ScoringSuperstructureAction() {}

    private ScoringSuperstructureAction withTargetElevatorExtensionFraction(double fraction) {
        this.targetElevatorExtensionFraction = fraction;
        return this;
    }

    private ScoringSuperstructureAction withTargetWristRotationFraction(double fraction) {
        this.targetWristRotationFraction = fraction;
        return this;
    }

    private ScoringSuperstructureAction withIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
        return this;
    }

    private ScoringSuperstructureAction stopIntakeOnGamePieceSeen() {
        this.endOnGamePieceSeen = true;
        return this;
    }

    private ScoringSuperstructureAction stopIntakeOnGamePieceNotSeen() {
        this.endOnGamePieceNotSeen = true;
        return this;
    }

    private ScoringSuperstructureAction withTrigger(Trigger trigger) {
        this.trigger = trigger;
        return this;
    }

    private ScoringSuperstructureAction requireWristTransition() {
        this.requiresWristTransition = true;
        return this;
    }

    private ScoringSuperstructureAction withStateAfter(ScoringSuperstructureAction stateAfter) {
        this.nextAction = stateAfter;
        return this;
    }

    public static ScoringSuperstructureAction HOLD(double elevatorProportion, double wristProportion) {
        return new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(elevatorProportion)
            .withTargetWristRotationFraction(wristProportion);
    }

    public static final ScoringSuperstructureAction
        IDLE = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.IDLE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_IDLE_Proportion)
            .stopIntakeOnGamePieceNotSeen(),
        INTAKE_FROM_HP = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.HP_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_HP_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_HP_Speed)
            .stopIntakeOnGamePieceSeen()
            .withTrigger(Controls.Operator.HPLoadingTrigger)
            .withStateAfter(IDLE),
        SCORE_L1_CORAL = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L1_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L1_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L1_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withTrigger(Controls.Operator.L1Trigger)
            .withStateAfter(IDLE),
        SCORE_L2_CORAL = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L2_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L2_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L2_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withTrigger(Controls.Operator.L2Trigger)
            .withStateAfter(IDLE),
        SCORE_L3_CORAL = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L3_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L3_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L3_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withTrigger(Controls.Operator.L3Trigger)
            .withStateAfter(IDLE),
        SCORE_L4_CORAL = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L4_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L4_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L4_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withTrigger(Controls.Operator.L4Trigger)
            .withStateAfter(IDLE),
        INTAKE_L2_ALGAE = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L2_ALGAE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L2_ALGAE_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L2_ALGAE_Speed)
            .requireWristTransition()
            .withTrigger(Controls.Driver.L2AlgaeTrigger)
            .withStateAfter(IDLE),
        INTAKE_L3_ALGAE = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L3_ALGAE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L3_ALGAE_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L3_ALGAE_Speed)
            .requireWristTransition()
            .withTrigger(Controls.Driver.L3AlgaeTrigger)
            .withStateAfter(IDLE),
        SCORE_BARGE = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.Barge_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_Barge_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_Barge_Speed)
            .requireWristTransition()
            .withTrigger(Controls.Driver.BargeScoringTrigger)
            .withStateAfter(IDLE),
        SCORE_PROCESSOR = new ScoringSuperstructureAction()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.Processor_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_Processor_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_Processor_Speed)
            .requireWristTransition()
            .withTrigger(Controls.Driver.ProcessorTrigger)
            .withStateAfter(IDLE);
}
