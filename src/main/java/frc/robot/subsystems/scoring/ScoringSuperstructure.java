package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.elevator.AbstractElevatorSubsystem;
import frc.robot.subsystems.scoring.elevator.ElevatorSysID;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

import java.util.Objects;
import java.util.OptionalDouble;
import java.util.function.Supplier;

public class ScoringSuperstructure extends SubsystemBase {
    private static ScoringSuperstructure instance;

    public static ScoringSuperstructure getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);
        return instance = Objects.requireNonNullElseGet(instance,
            ScoringSuperstructure::new
        );
    }

    private final AbstractElevatorSubsystem elevator;
    private final AbstractEndEffectorSubsystem endEffector;

    private ScoringSuperstructureAction currentAction = ScoringSuperstructureAction.IDLE;
    private ScoringSuperstructureAction prevAction = ScoringSuperstructureAction.IDLE;
    private ScoringSuperstructureState currentState = ScoringSuperstructureState.EXECUTING_ACTION;

    private boolean isManualControlEnabled = false;

    public ScoringSuperstructure() {
        this.elevator = AbstractElevatorSubsystem.getInstance();
        this.endEffector = AbstractEndEffectorSubsystem.getInstance();
        SmartDashboard.putBoolean("isManualControlEnabled", isManualControlEnabled);
    }

    private void setState(ScoringSuperstructureAction action) {
        if (action != this.currentAction) {
            prevAction = this.currentAction;
            this.currentAction = action;

            boolean requiresWristTransition = prevAction.requiresWristTransition || action.requiresWristTransition;
            if (requiresWristTransition) {
                currentState = ScoringSuperstructureState.TRANSITION_BEFORE_ELEVATOR;
            } else {
                currentState = ScoringSuperstructureState.ELEVATOR_MOVE_NO_TRANSITION;
            }
        }
    }

    /**
     * @param state the new state to set the scoring superstructure to.
     *
     * @return an instantaneous command that sets the state to {@param state}. It does not run the scoring
     * superstructure, only sets the state.
     */
    public Command setScoringState(ScoringSuperstructureAction state) {
        return setScoringState(() -> state);
    }

    /**
     * @param state the new state to set the scoring superstructure to.
     *
     * @return an instantaneous command that sets the state to {@param state}. It does not run the scoring
     * superstructure, only sets the state.
     */
    public Command setScoringState(Supplier<ScoringSuperstructureAction> state) {
        return Commands.runOnce(
            () -> setState(state.get())
        );
    }

    public Command hold() {
        return setScoringState(() -> ScoringSuperstructureAction.HOLD(
            elevator.getCurrentExtensionFraction(),
            ScoringConstants.EndEffectorConstants.ProportionToPosition.convertBackwards(endEffector.getCurrentPosition())
        ));
    }

    /**
     * @return command to run the scoring subsystem. It takes into account which "sub-subsystem" is the last to move.
     */
    public Command runScoringState() {
        return Commands.run(
            () -> {
                switch (currentState) {
                    case TRANSITION_BEFORE_ELEVATOR -> {
                        if (endEffector.isWristAtTarget()) {
                            currentState = currentState.next();
                        }
                    }
                    case ELEVATOR_MOVE_WITH_TRANSITION -> {
                        if (elevator.isAtTarget()) {
                            currentState = currentState.next();
                        }
                    }
                    case TRANSITION_AFTER_ELEVATOR -> {
                        if (endEffector.isWristAtTarget()) {
                            currentState = currentState.next();
                        }
                    }
                    case ELEVATOR_MOVE_NO_TRANSITION -> {
                        if (elevator.isAtTarget()) {
                            currentState = currentState.next();
                        }
                    }
                    case EXECUTING_ACTION -> {
                        boolean isActionComplete = currentAction.shouldStopIntakeOnGamePieceSeen && endEffector.hasCoral()
                            || currentAction.shouldStopIntakeOnGamePieceNotSeen && !endEffector.hasCoral();
                        if (isActionComplete) {
                            currentAction = currentAction.nextAction;
                        }
                    }
                    case DONE -> {
                        currentAction = currentAction.nextAction;
                    }
                }

                OptionalDouble targetElevator = currentState.getTargetElevatorExtensionFraction(currentAction);
                OptionalDouble targetWrist = currentState.getTargetWristRotationFraction(currentAction);

                elevator.setTargetExtensionProportion(targetElevator.orElse(elevator.getCurrentExtensionFraction()));
                endEffector.setTargetRotationFraction(targetWrist.orElse(endEffector.getCurrentRotationFraction()));
            },
            this
        );
    }

    public Command toggleManualControl() {
        return runOnce(() -> {
            this.isManualControlEnabled = !this.isManualControlEnabled;
            this.elevator.setManualControlEnabled(isManualControlEnabled);
            this.endEffector.setManualControlEnabled(isManualControlEnabled);
        });
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
        return elevator.isAtTarget() && endEffector.isHopperStateFinished();
    }

    public Trigger isStateFinished = new Trigger(this::isStateFinished);

    /**
     * automatically detects whether current state was "aborted" or is finished, then sets the current state to the next
     * state if so.
     */
    @Override
    public void periodic() {
        if (isStateFinished()) {
            setState(currentAction.nextAction);
        }
        // In teleop, stop the current state if the control is no longer active
        else if (RobotState.isTeleop() && !currentAction.trigger.getAsBoolean()) {
            setState(ScoringSuperstructureAction.IDLE);
        }
        // Sets scoring mechanisms to IDLE in case robot acceleration is high.
        if (SubsystemManager.getInstance().getDrivetrain().getAccelerationInGs() >= .4) {
            setState(ScoringSuperstructureAction.IDLE);
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
}