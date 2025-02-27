package frc.robot.subsystems.scoring;

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
    public String name;
    /**
     * The state to use after this state has finished. Usually, this will
     * be the idle state
     */
    public ScoringSuperstructureAction nextAction = this;

    private ScoringSuperstructureAction(String name) {
        this.name = name;
    }

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

    private ScoringSuperstructureAction requireWristTransition() {
        this.requiresWristTransition = true;
        return this;
    }

    private ScoringSuperstructureAction withStateAfter(ScoringSuperstructureAction stateAfter) {
        this.nextAction = stateAfter;
        return this;
    }

    public static ScoringSuperstructureAction HOLD(double elevatorProportion, double wristProportion) {
        return new ScoringSuperstructureAction("HOLD")
            .withTargetElevatorExtensionFraction(elevatorProportion)
            .withTargetWristRotationFraction(wristProportion);
    }

    public static final ScoringSuperstructureAction
        IDLE = new ScoringSuperstructureAction("IDLE")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.IDLE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_IDLE_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_Idle_Speed)
            .stopIntakeOnGamePieceNotSeen(),
        IDLE_STOW_ALGAE = new ScoringSuperstructureAction("IDLE_STOW_ALGAE")
                    .withTargetElevatorExtensionFraction(ElevatorSetpoints.IDLE_Proportion)
                    .withTargetWristRotationFraction(WristSetpoints.Wrist_ALGAESTOW_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_Idle_Speed)
                    .stopIntakeOnGamePieceNotSeen(),
        INTAKE_FROM_HP = new ScoringSuperstructureAction("INTAKE_FROM_HP")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.HP_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_HP_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_HP_Speed)
            .stopIntakeOnGamePieceSeen()
            .withStateAfter(IDLE),
        SCORE_L1_CORAL = new ScoringSuperstructureAction("SCORE_L1_CORAL")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L1_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L1_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L1_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withStateAfter(IDLE),
        SCORE_L2_CORAL = new ScoringSuperstructureAction("SCORE_L2_CORAL")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L2_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L2_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L2_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withStateAfter(IDLE),
        SCORE_L3_CORAL = new ScoringSuperstructureAction("SCORE_L3_CORAL")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L3_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L3_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L3_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withStateAfter(IDLE),
        SCORE_L4_CORAL = new ScoringSuperstructureAction("SCORE_L4_CORAL")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L4_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L4_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L4_Speed)
            .stopIntakeOnGamePieceNotSeen()
            .requireWristTransition()
            .withStateAfter(IDLE),
        INTAKE_L2_ALGAE = new ScoringSuperstructureAction("INTAKE_L2_ALGAE")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L2_ALGAE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L2_ALGAE_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L2_ALGAE_Speed)
            .requireWristTransition()
            .withStateAfter(IDLE_STOW_ALGAE),
        INTAKE_L3_ALGAE = new ScoringSuperstructureAction("INTAKE_L3_ALGAE")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L3_ALGAE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L3_ALGAE_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_L3_ALGAE_Speed)
            .requireWristTransition()
            .withStateAfter(IDLE_STOW_ALGAE),
        SCORE_BARGE = new ScoringSuperstructureAction("SCORE_BARGE")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.Barge_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_Barge_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_Barge_Speed)
            .requireWristTransition()
            .withStateAfter(IDLE),
        SCORE_PROCESSOR = new ScoringSuperstructureAction("SCORE_PROCESSOR")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.Processor_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_Processor_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_Processor_Speed)
            .requireWristTransition()
            .withStateAfter(IDLE),
        GROUND_INTAKE = new ScoringSuperstructureAction("GROUND_INTAKE")
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.Ground_Intake_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_Ground_Intake_Proportion)
            .withIntakeSpeed(1.0)
            .withStateAfter(IDLE_STOW_ALGAE);

    public String toString() {
        return name;
    }
}
