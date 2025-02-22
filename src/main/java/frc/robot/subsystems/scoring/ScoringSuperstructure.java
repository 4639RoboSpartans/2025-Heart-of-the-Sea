package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Controls;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.elevator.AbstractElevatorSubsystem;
import frc.robot.subsystems.scoring.elevator.ElevatorSysID;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.BiPredicate;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public final class ScoringSuperstructure extends SubsystemBase {
    private static ScoringSuperstructure instance;

    public static ScoringSuperstructure getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);
        return instance = Objects.requireNonNullElseGet(instance,
            () -> new ScoringSuperstructure(access)
        );
    }

    private final AbstractElevatorSubsystem elevator;
    private final AbstractEndEffectorSubsystem endEffector;

    private ScoringSuperstructureAction currentAction = ScoringSuperstructureAction.IDLE;
    private ScoringSuperstructureState currentState = ScoringSuperstructureState.EXECUTING_ACTION;
    private double elevatorAdjustment = 0;
    private double wristAdjustment = 0;

    private boolean isManualControlEnabled = false;

    public ScoringSuperstructure(SubsystemManager.GetInstanceAccess access) {
        this.elevator = AbstractElevatorSubsystem.getInstance(access);
        this.endEffector = AbstractEndEffectorSubsystem.getInstance(access);
        SmartDashboard.putBoolean("isManualControlEnabled", isManualControlEnabled);
    }

    private void setCurrentAction(ScoringSuperstructureAction action) {
        if (action != this.currentAction) {
            ScoringSuperstructureAction prevAction = this.currentAction;
            this.currentAction = action;

            boolean requiresWristTransition = prevAction.requiresWristTransition || action.requiresWristTransition;
            if (requiresWristTransition) {
                currentState = ScoringSuperstructureState.TRANSITION_BEFORE_ELEVATOR;
            } else {
                currentState = ScoringSuperstructureState.ELEVATOR_MOVE_NO_TRANSITION;
            }
            resetAdjustments();
        }
    }

    private void resetAdjustments() {
        elevatorAdjustment = 0;
        wristAdjustment = 0;
    }

    /**
     * @param state the new state to set the scoring superstructure to.
     *
     * @return an instantaneous command that sets the state to {@param state}. It does not run the scoring
     * superstructure, only sets the state.
     */
    public Command setAction(ScoringSuperstructureAction state) {
        return setAction(() -> state);
    }

    /**
     * @param state the new state to set the scoring superstructure to.
     *
     * @return an instantaneous command that sets the state to {@param state}. It does not run the scoring
     * superstructure, only sets the state.
     */
    public Command setAction(Supplier<ScoringSuperstructureAction> state) {
        return Commands.runOnce(
            () -> setCurrentAction(state.get())
        );
    }

    public Command hold() {
        return setAction(() -> ScoringSuperstructureAction.HOLD(
            elevator.getCurrentExtensionFraction(),
            ScoringConstants.EndEffectorConstants.RotationFractionToMotorPosition.convertBackwards(endEffector.getCurrentMotorPosition())
        ));
    }

    /**
     * @return command to run the scoring subsystem. It takes into account which "sub-subsystem" is the last to move.
     */
    public Command runScoringState() {
        return run(() -> {
            if (isManualControlEnabled) runManualPeriodic();
            else runActionPeriodic();

            SmartDashboard.putString("Scoring superstructure info",
                "%s: ele = [curr %.3f; tgt %.3f]; wst = [curr %.3f; tgt %.3f]".formatted(
                    currentState,
                    elevator.getCurrentPosition(),
                    elevator.getTargetPosition(),
                    endEffector.getCurrentMotorPosition(),
                    endEffector.getTargetPosition()
                )
            );
        });
    }

    public Command elevatorHoningCommand() {
        return run(() ->
                elevator.setRawMotorVoltage(Voltage.ofBaseUnits(-0.8, Units.Volt))
                )
                .until(elevator::shouldStopRunningHoningCommand)
                .onlyIf(this::isManualControlEnabled);
    }

    private void runManualPeriodic() {
        double currentTargetElevatorExtensionFraction = elevator.getTargetExtensionFraction();
        double targetElevatorExtensionFraction = currentTargetElevatorExtensionFraction
            + Controls.Operator.ManualControlElevator.getAsDouble() * 0.03;

        elevator.setTargetExtensionFraction(MathUtil.clamp(targetElevatorExtensionFraction, 0, 1));
        endEffector.setTargetWristRotationFraction(Controls.Operator.ManualControlWrist.getAsDouble());
        endEffector.setIntakeSpeed(Controls.Operator.ManualControlIntake.getAsDouble());
    }

    private void runActionPeriodic() {
        // If the current action's trigger is no longer active,
        // move to the next action
        if (!currentAction.trigger.getAsBoolean()) {
            setCurrentAction(currentAction.nextAction);
        }

        // Get new setpoints
        double targetElevatorExtensionFraction = currentState
            .getTargetElevatorExtensionFraction(currentAction)
            .orElseGet(elevator::getTargetExtensionFraction);
        double targetWristRotationFraction = currentState
            .getTargetWristRotationFraction(currentAction)
            .orElseGet(endEffector::getTargetRotationFraction);

//        double intakeSpeed = currentState.getIntakeSpeed(currentAction);
        double intakeSpeed = Controls.Operator.ManualControlIntake.getAsDouble();

        // Update fine-tuning offsets
        elevatorAdjustment += 0.002 * Controls.Operator.MicroElevatorAdjustment.getAsDouble();
        wristAdjustment += 0.006 * Controls.Operator.MicroWristAdjustment.getAsDouble();
        elevatorAdjustment = MathUtil.clamp(
            targetElevatorExtensionFraction + elevatorAdjustment,
            0, 1
        ) - targetElevatorExtensionFraction;
        wristAdjustment = MathUtil.clamp(
            targetWristRotationFraction + wristAdjustment,
            0, 1
        ) - targetWristRotationFraction;

        elevator.setTargetExtensionFraction(targetElevatorExtensionFraction + elevatorAdjustment);
        endEffector.setTargetWristRotationFraction(targetWristRotationFraction + wristAdjustment);
        endEffector.setIntakeSpeed(intakeSpeed);

        // Advance the state if necessary
        if (currentState.shouldAdvanceState(currentAction, endEffector, elevator)) {
            currentState = currentState.next();
            resetAdjustments();
        }
        // If the state is finished, go to the next action
        if (currentState == ScoringSuperstructureState.DONE) {
            setCurrentAction(currentAction.nextAction);
        }
    }

    public Command toggleManualControl() {
        return runOnce(() -> this.isManualControlEnabled = !this.isManualControlEnabled);
    }

    /**
     * @return whether both "sub-subsystems" at the specified position
     */
    public boolean isAtPosition() {
        return elevator.isAtTarget() && endEffector.isWristAtTarget();
    }

    public Trigger isAtPosition = new Trigger(this::isAtPosition);

    /**
     * @return whether the state is finished. Note this is different from {@link ScoringSuperstructure#isAtPosition()},
     * since the state could only be finished when a game piece is detected, but we might only start the intake wheels when
     * {@link ScoringSuperstructure#isAtPosition()} returns true.
     */
    public boolean isStateFinished() {
        return elevator.isAtTarget() && endEffector.isWristAtTarget();
    }

    public Trigger isStateFinished = new Trigger(this::isStateFinished);

    /**
     * automatically detects whether current state was "aborted" or is finished, then sets the current state to the next
     * state if so.
     */
    @Override
    public void periodic() {
        // Sets scoring mechanisms to IDLE in case robot acceleration is high.
        if (SubsystemManager.getInstance().getDrivetrain().getAccelerationInGs() >= .4) {
            setCurrentAction(ScoringSuperstructureAction.IDLE);
        }

        SmartDashboard.putBoolean("isManualControlEnabled", isManualControlEnabled);
    }

    /**
     * @return the real life length of the elevator, for use in simulation only.
     */
    public Distance getCurrentElevatorLength() {
        return elevator.getCurrentHeight();
    }

    /**
     * @return the real life target length of the elevator, for use in simulation only.
     */
    public Distance getTargetElevatorLength() {
        return elevator.getTargetHeight();
    }

    /**
     * @return the real life rotation of the wrist, for use in simulation only.
     */
    public Rotation2d getCurrentWristRotation() {
        return endEffector.getCurrentRotation();
    }

    /**
     * @return the real life target rotation of the wrist, for use in simulation only.
     */
    public Rotation2d getTargetWristRotation() {
        return endEffector.getTargetRotation();
    }

    public Command elevatorSysIDQuasistatic(SysIdRoutine.Direction direction) {
        return ElevatorSysID.sysIdQuasistatic(direction);
    }

    public Command elevatorSysIDDynamic(SysIdRoutine.Direction direction) {
        return ElevatorSysID.sysIdDynamic(direction);
    }

    public boolean isManualControlEnabled() {
        return isManualControlEnabled;
    }

    public AbstractEndEffectorSubsystem getEndEffectorSubsystem() {
        return endEffector;
    }

    public AbstractElevatorSubsystem getElevatorSubsystem() {
        return elevator;
    }

    public ScoringSuperstructureAction getCurrentAction() {
        return currentAction;
    }
}