package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.IntakeSpeeds;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.WristSetpoints;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class ScoringSuperstructureAction {
    public static boolean useInterpolatingSetpoints = true;

    public DoubleSupplier targetElevatorExtensionFraction = () -> 0;
    public DoubleSupplier targetWristRotationFraction = () -> 0;
    public double intakeSpeed = 0;
    public boolean endOnGamePieceSeen = false;
    public boolean endOnGamePieceNotSeen = false;
    public boolean requiresWristTransition = false;
    public boolean useManualControlInTeleop = true;
    public String name;

    private static final AbstractSwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();
    /**
     * The state to use after this state has finished. Usually, this will
     * be the idle state
     */
    public ScoringSuperstructureAction nextAction = this;

    private ScoringSuperstructureAction(ScoringSuperstructureAction action, String name) {
        this.targetElevatorExtensionFraction = action.targetElevatorExtensionFraction;
        this.targetWristRotationFraction = action.targetWristRotationFraction;
        this.intakeSpeed = action.intakeSpeed;
        this.endOnGamePieceSeen = action.endOnGamePieceSeen;
        this.endOnGamePieceNotSeen = action.endOnGamePieceNotSeen;
        this.requiresWristTransition = action.requiresWristTransition;
        this.useManualControlInTeleop = action.useManualControlInTeleop;
        this.name = name;
    }

    private ScoringSuperstructureAction(ScoringSuperstructureAction action) {
        this(action, action.name);
    }

    private ScoringSuperstructureAction(String name) {
        this.name = name;
    }

    private ScoringSuperstructureAction modifyInPlace(Consumer<ScoringSuperstructureAction> f) {
        f.accept(this);
        return this;
    }

    private ScoringSuperstructureAction withTargetElevatorExtensionFraction(DoubleSupplier fraction) {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.targetElevatorExtensionFraction = fraction);
    }

    private ScoringSuperstructureAction withTargetWristRotationFraction(DoubleSupplier fraction) {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.targetWristRotationFraction = fraction);
    }

    private ScoringSuperstructureAction withIntakeSpeed(double speed) {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.intakeSpeed = speed);
    }

    private ScoringSuperstructureAction stopIntakeOnGamePieceSeen() {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.endOnGamePieceSeen = true);
    }

    private ScoringSuperstructureAction stopIntakeOnGamePieceNotSeen() {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.endOnGamePieceNotSeen = true);
    }

    private ScoringSuperstructureAction requireWristTransition() {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.requiresWristTransition = true);
    }

    private ScoringSuperstructureAction useManualControlInTeleop(boolean useManualControlInTeleop) {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.useManualControlInTeleop = useManualControlInTeleop);
    }

    private ScoringSuperstructureAction withStateAfter(ScoringSuperstructureAction stateAfter) {
        return new ScoringSuperstructureAction(this).modifyInPlace(a -> a.nextAction = stateAfter);
    }

    public static ScoringSuperstructureAction HOLD(double elevatorProportion, double wristProportion) {
        return new ScoringSuperstructureAction("HOLD")
                .withTargetElevatorExtensionFraction(() -> elevatorProportion)
                .withTargetWristRotationFraction(() -> wristProportion);
    }

    public ScoringSuperstructureAction withNoIntakeUsage() {
        return new ScoringSuperstructureAction(this, name + "_NO_INTAKE").modifyInPlace(a -> {
            a.endOnGamePieceNotSeen = false;
            a.endOnGamePieceSeen = false;
            a.intakeSpeed = 0;
            a.requiresWristTransition = false;
            a.name += "NO_INTAKE";
        });
    }

    public static final ScoringSuperstructureAction
            IDLE = new ScoringSuperstructureAction("IDLE")
            .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.IDLE_Proportion)
            .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_IDLE_Proportion)
            .withIntakeSpeed(IntakeSpeeds.Intake_Idle_Speed)
            .useManualControlInTeleop(true),
            IDLE_STOW_ALGAE = new ScoringSuperstructureAction("IDLE_STOW_ALGAE")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.IDLE_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_ALGAESTOW_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_Idle_Speed)
                    .stopIntakeOnGamePieceNotSeen(),
            INTAKE_FROM_HP = new ScoringSuperstructureAction("INTAKE_FROM_HP")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.HP_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_HP_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_HP_Speed / 2)
                    .withStateAfter(IDLE)
                    .useManualControlInTeleop(false),
            INTAKE_FROM_HP_LOWER = new ScoringSuperstructureAction("INTAKE_FROM_HP_LOWER")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.HP_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_HP_Lower_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_HP_Speed)
                    .stopIntakeOnGamePieceSeen()
                    .withStateAfter(IDLE)
                    .useManualControlInTeleop(false),
            SCORE_L1_CORAL = new ScoringSuperstructureAction("SCORE_L1_CORAL")
                    .withTargetElevatorExtensionFraction(() -> {
                        double res = ElevatorSetpoints.L1_Proportion + (drivetrain.getDistanceFromReefFace() - 387.5) * 0.0001;
                        return res > 1 && useInterpolatingSetpoints ? ElevatorSetpoints.L1_Proportion : res;
                    })
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_L1_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_L1_Speed)
                    .stopIntakeOnGamePieceNotSeen()
                    .requireWristTransition()
                    .withStateAfter(IDLE),
            SCORE_L2_CORAL = new ScoringSuperstructureAction("SCORE_L2_CORAL")
                    .withTargetElevatorExtensionFraction(() -> {
                        double res = ElevatorSetpoints.L2_Proportion + (drivetrain.getDistanceFromReefFace() - 387.5) * 0.0001;
                        return res > 1 && useInterpolatingSetpoints ? ElevatorSetpoints.L2_Proportion : res;
                    })
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_L2_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_L2_Speed)
                    .stopIntakeOnGamePieceNotSeen()
                    .requireWristTransition()
                    .withStateAfter(IDLE),
            SCORE_L3_CORAL = new ScoringSuperstructureAction("SCORE_L3_CORAL")
                    .withTargetElevatorExtensionFraction(() -> {
                        double res = ElevatorSetpoints.L3_Proportion + (drivetrain.getDistanceFromReefFace() - 387.5) * 0.0001;
                        return res > 1 && useInterpolatingSetpoints ? ElevatorSetpoints.L3_Proportion : res;
                    })
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_L3_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_L3_Speed)
                    .stopIntakeOnGamePieceNotSeen()
                    .requireWristTransition()
                    .withStateAfter(IDLE),
            SCORE_L4_CORAL = new ScoringSuperstructureAction("SCORE_L4_CORAL")
                    .withTargetElevatorExtensionFraction(() -> {
                        double res = ElevatorSetpoints.L4_Proportion + (drivetrain.getDistanceFromReefFace() - 387.5) * 0.0001;
                        return res > 1 && useInterpolatingSetpoints ? ElevatorSetpoints.L4_Proportion : res;
                    })
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_L4_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_L4_Speed)
                    .stopIntakeOnGamePieceNotSeen()
                    .requireWristTransition()
                    .withStateAfter(IDLE),
            INTAKE_L2_ALGAE = new ScoringSuperstructureAction("INTAKE_L2_ALGAE")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.L2_ALGAE_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_L2_ALGAE_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_L2_ALGAE_Speed)
                    .requireWristTransition()
                    .withStateAfter(IDLE_STOW_ALGAE),
            INTAKE_L3_ALGAE = new ScoringSuperstructureAction("INTAKE_L3_ALGAE")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.L3_ALGAE_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_L3_ALGAE_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_L3_ALGAE_Speed)
                    .requireWristTransition()
                    .withStateAfter(IDLE_STOW_ALGAE),
            SCORE_BARGE = new ScoringSuperstructureAction("SCORE_BARGE")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.Barge_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_Barge_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_Barge_Speed)
                    .requireWristTransition()
                    .withStateAfter(IDLE),
            SCORE_PROCESSOR = new ScoringSuperstructureAction("SCORE_PROCESSOR")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.Processor_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_Processor_Proportion)
                    .withIntakeSpeed(IntakeSpeeds.Intake_Processor_Speed)
                    .requireWristTransition()
                    .withStateAfter(IDLE),
            GROUND_INTAKE = new ScoringSuperstructureAction("GROUND_INTAKE")
                    .withTargetElevatorExtensionFraction(() -> ElevatorSetpoints.Ground_Intake_Proportion)
                    .withTargetWristRotationFraction(() -> WristSetpoints.Wrist_Ground_Intake_Proportion)
                    .withIntakeSpeed(1.0)
                    .withStateAfter(IDLE_STOW_ALGAE);

    public String toString() {
        return name;
    }

    public static Command toggleInterpolatingSetpoints() {
        return Commands.runOnce(
                () -> useInterpolatingSetpoints = !useInterpolatingSetpoints
        );
    }
}
