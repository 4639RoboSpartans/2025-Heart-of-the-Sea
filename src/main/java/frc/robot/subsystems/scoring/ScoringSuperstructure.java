package frc.robot.subsystems.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Controls;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.WristSetpoints;
import frc.robot.subsystems.scoring.elevator.AbstractElevatorSubsystem;
import frc.robot.subsystems.scoring.elevator.ElevatorSysID;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

import java.util.Objects;
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

            boolean requiresWristTransition = prevAction.requiresWristTransition || currentAction.requiresWristTransition;
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
        });
    }

    public Command elevatorHomingCommand() {
        double ELEVATOR_HOMING_INITIAL_OFFSET = 0.01;
        double ELEVATOR_HOMING_SPEED = 0.007;
        double ELEVATOR_HOMING_MAX_OFFSET = 0.01;
        return startRun(
            () -> {
                elevator.setTargetExtensionFraction(elevator.getCurrentExtensionFraction() + ELEVATOR_HOMING_INITIAL_OFFSET);
                endEffector.setTargetWristRotationFraction(0.5);
            },
            () -> elevator.setTargetExtensionFraction(
                Math.max(
                    elevator.getTargetExtensionFraction() - ELEVATOR_HOMING_SPEED,
                    elevator.getCurrentExtensionFraction() - ELEVATOR_HOMING_MAX_OFFSET
                )
            )
        ).until(elevator::isPhysicallyStopped).finallyDo((wasInterrupted) -> {
            if (!wasInterrupted) {
                elevator.resetCurrentExtensionFractionTo(ElevatorSetpoints.Homing_Proportion);
                setCurrentAction(ScoringSuperstructureAction.IDLE);
            }
        }).withName("elevatorHomingCommand");
    }

    private void runManualPeriodic() {
        double currentTargetElevatorExtensionFraction = elevator.getTargetExtensionFraction();
        double targetElevatorExtensionFraction = currentTargetElevatorExtensionFraction
            + Controls.Operator.ManualControlElevator.getAsDouble() * 0.03;

        elevator.setTargetExtensionFraction(MathUtil.clamp(targetElevatorExtensionFraction, ElevatorSetpoints.IDLE_Proportion, 1));
        endEffector.setTargetWristRotationFraction(MathUtil.clamp(
            Controls.Operator.ManualControlWrist.getAsDouble(),
            WristSetpoints.Wrist_IDLE_Proportion,
            1.2
        ));
        endEffector.setIntakeSpeed(Controls.Operator.ManualControlIntake.getAsDouble());
    }

    private void runActionPeriodic() {
        // Get new setpoints
        double targetElevatorExtensionFraction = currentState
            .getTargetElevatorExtensionFraction(currentAction)
            .orElseGet(elevator::getTargetExtensionFraction);
        double targetWristRotationFraction = currentState
            .getTargetWristRotationFraction(currentAction)
            .orElseGet(endEffector::getTargetRotationFraction);
        double manualIntakeSpeed = Controls.Operator.ManualControlIntake.getAsDouble() * Math.abs(currentAction.intakeSpeed);
        double intakeSpeed = (
            RobotState.isTeleop() && currentAction.useManualControlInTeleop
                ? manualIntakeSpeed
                : manualIntakeSpeed == 0 ? currentState.getIntakeSpeed(currentAction) : manualIntakeSpeed
        );

        // Update fine-tuning offsets
        elevatorAdjustment += 0.002 * Controls.Operator.MicroElevatorAdjustment.getAsDouble();
        wristAdjustment += 0.006 * Controls.Operator.MicroWristAdjustment.getAsDouble();
        elevatorAdjustment = MathUtil.clamp(
            targetElevatorExtensionFraction + elevatorAdjustment,
            ElevatorSetpoints.IDLE_Proportion, 1
        ) - targetElevatorExtensionFraction;
        wristAdjustment = MathUtil.clamp(
            targetWristRotationFraction + wristAdjustment,
            WristSetpoints.Wrist_Lowest_Proportion, 1
        ) - targetWristRotationFraction;

        // Set speeds
        elevator.setTargetExtensionFraction(targetElevatorExtensionFraction + elevatorAdjustment);
        endEffector.setTargetWristRotationFraction(targetWristRotationFraction + wristAdjustment);
        endEffector.setIntakeSpeed(intakeSpeed);

        // Advance the state if necessary
        SmartDashboard.putBoolean("Should Advance State", currentState.shouldAdvanceState(currentAction, endEffector, elevator));
        if (currentState.shouldAdvanceState(currentAction, endEffector, elevator)) {
            currentState = currentState.next();
            resetAdjustments();
        }

        // If the state is finished and, go to the next action, requires IDLE trigger to go down to IDLE
        if (currentState == ScoringSuperstructureState.DONE && RobotState.isAutonomous()) {
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

    public boolean isAtActionPosition() {
        return elevator.isAtTarget() && endEffector.isWristAtActionTarget();
    }

    public Trigger isAtPosition = new Trigger(this::isAtPosition);

    /**
     * @return whether the state is finished. Note this is different from {@link ScoringSuperstructure#isAtPosition()},
     * since the state could only be finished when a game piece is detected, but we might only start the intake wheels when
     * {@link ScoringSuperstructure#isAtPosition()} returns true.
     */
    public boolean isStateFinished() {
        return elevator.isAtTarget() && endEffector.isWristAtActionTarget();
    }

    public Trigger isStateFinished = new Trigger(this::isStateFinished);

    /**
     * automatically detects whether current state was "aborted" or is finished, then sets the current state to the next
     * state if so.
     */
    @Override
    public void periodic() {
        // Sets scoring mechanisms to IDLE in case robot acceleration is high.
        if (SubsystemManager.getInstance().getDrivetrain().getAccelerationInGs() >= 1.0 / elevator.getCurrentExtensionFraction()) {
            setCurrentAction(ScoringSuperstructureAction.IDLE);
        }
        SmartDashboard.putNumber("Elevator Fraction", elevator.getCurrentExtensionFraction());
        SmartDashboard.putNumber("Wrist Position", endEffector.getCurrentMotorPosition());

        SmartDashboard.putString("State", currentState.name());
        SmartDashboard.putString("Action", currentAction.toString());

        SmartDashboard.putBoolean("is in auton", RobotState.isAutonomous());
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

    public ScoringSuperstructureState getCurrentState() {
        return currentState;
    }
}