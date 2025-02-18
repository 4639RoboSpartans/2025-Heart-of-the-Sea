package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.WristSetpoints;

import java.util.OptionalDouble;

public class ScoringSuperstructureState {
    public double targetElevatorExtensionFraction = 0;
    public double targetWristRotationFraction = 0;
    public double intakeSpeed = 0;
    public boolean shouldStopIntakeOnGamePieceSeen = false;
    public boolean shouldStopIntakeOnGamePieceNotSeen = false;

    /**
     * Some states will require the wrist to rotate outward while the
     * elevator moves in order to prevent the end effector from
     * colliding with the elevator. In these cases, this rotation
     * fraction is used before the elevator starts extending and after
     * the elevator has reached its target position
     */
    public OptionalDouble transitionWristRotationFraction = OptionalDouble.empty();
    /** The trigger used to activate the state */
    public Trigger trigger = new Trigger(() -> true);
    /**
     * The state to use after this state has finished. Usually, this will
     * be the idle state
     */
    public ScoringSuperstructureState stateAfter = this;

    private ScoringSuperstructureState() {}

    private ScoringSuperstructureState withTargetElevatorExtensionFraction(double fraction) {
        this.targetElevatorExtensionFraction = fraction;
        return this;
    }

    private ScoringSuperstructureState withTargetWristRotationFraction(double fraction) {
        this.targetWristRotationFraction = fraction;
        return this;
    }

    private ScoringSuperstructureState withIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
        return this;
    }

    private ScoringSuperstructureState stopIntakeOnGamePieceSeen() {
        this.shouldStopIntakeOnGamePieceSeen = true;
        return this;
    }

    private ScoringSuperstructureState stopIntakeOnGamePieceNotSeen() {
        this.shouldStopIntakeOnGamePieceNotSeen = true;
        return this;
    }

    private ScoringSuperstructureState withTrigger(Trigger trigger) {
        this.trigger = trigger;
        return this;
    }

    private ScoringSuperstructureState withTransitionWristRotationFraction(double transitionWristRotationFraction) {
        this.transitionWristRotationFraction = OptionalDouble.of(transitionWristRotationFraction);
        return this;
    }

    private ScoringSuperstructureState withStateAfter(ScoringSuperstructureState stateAfter) {
        this.stateAfter = stateAfter;
        return this;
    }

    public static ScoringSuperstructureState HOLD(double elevatorProportion, double wristProportion) {
        return new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(elevatorProportion)
            .withTargetWristRotationFraction(wristProportion);
    }

    public static final ScoringSuperstructureState
        IDLE = new ScoringSuperstructureState()
        .withTargetElevatorExtensionFraction(ElevatorSetpoints.IDLE_Proportion)
        .withTargetWristRotationFraction(WristSetpoints.Wrist_IDLE_Proportion)
        .stopIntakeOnGamePieceNotSeen()
        .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion),
        HP_LOADING = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.HP_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_HP_Proportion)
            .withIntakeSpeed(0.5)
            .stopIntakeOnGamePieceSeen()
            .withTrigger(Controls.Operator.HPLoadingTrigger)
            .withStateAfter(IDLE),
        L1 = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L1_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L1_Proportion)
            .withIntakeSpeed(0.5)
            .stopIntakeOnGamePieceNotSeen()
            .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion)
            .withTrigger(Controls.Operator.L1Trigger)
            .withStateAfter(IDLE),
        L2 = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L2_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L2_Proportion)
            .withIntakeSpeed(0.5)
            .stopIntakeOnGamePieceNotSeen()
            .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion)
            .withTrigger(Controls.Operator.L2Trigger)
            .withStateAfter(IDLE),
        L3 = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L3_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L3_Proportion)
            .withIntakeSpeed(0.5)
            .stopIntakeOnGamePieceNotSeen()
            .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion)
            .withTrigger(Controls.Operator.L3Trigger)
            .withStateAfter(IDLE),
        L4 = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L4_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L4_Proportion)
            .withIntakeSpeed(0.5)
            .stopIntakeOnGamePieceNotSeen()
            .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion)
            .withTrigger(Controls.Operator.L4Trigger)
            .withStateAfter(IDLE),
        L2_ALGAE = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L2_ALGAE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L2_ALGAE_Proportion)
            .withIntakeSpeed(-0.5)
            .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion)
            .withTrigger(Controls.Operator.L2AlgaeTrigger)
            .withStateAfter(IDLE),
        L3_ALGAE = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.L3_ALGAE_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_L3_ALGAE_Proportion)
            .withIntakeSpeed(-0.5)
            .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion)
            .withTrigger(Controls.Operator.L3AlgaeTrigger)
            .withStateAfter(IDLE),
        BARGE_SCORING = new ScoringSuperstructureState()
            .withTargetElevatorExtensionFraction(ElevatorSetpoints.Barge_Proportion)
            .withTargetWristRotationFraction(WristSetpoints.Wrist_Barge_Proportion)
            .withIntakeSpeed(1)
            .withTransitionWristRotationFraction(WristSetpoints.Wrist_Transition_Proportion)
            .withTrigger(Controls.Operator.BargeScoringTrigger)
            .withStateAfter(IDLE);
}
