package frc.robot.subsystems.scoring;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.WristSetpoints;
import frc.robot.subsystems.scoring.elevator.AbstractElevatorSubsystem;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public enum ScoringSuperstructureState {
    TRANSITION_BEFORE_ELEVATOR,
    ELEVATOR_MOVE_WITH_TRANSITION,
    TRANSITION_AFTER_ELEVATOR,
    ELEVATOR_MOVE_NO_TRANSITION,
    EXECUTING_ACTION,
    DONE;

    public boolean shouldAdvanceState(
        ScoringSuperstructureAction action,
        AbstractEndEffectorSubsystem endEffector,
        AbstractElevatorSubsystem elevator
    ) {
        return switch (this) {
            case TRANSITION_BEFORE_ELEVATOR -> {
                if (SubsystemManager.getInstance().getScoringSuperstructure().elevatorSkipTransitionThreshold()) yield endEffector.isWristAtTarget();
                else yield true;
            }
            case ELEVATOR_MOVE_WITH_TRANSITION -> elevator.isNearTarget();
            case TRANSITION_AFTER_ELEVATOR -> ScoringConstants.autonShouldAdvanceToOuttakeTrigger.getAsBoolean() && elevator.isAtTarget();
            case ELEVATOR_MOVE_NO_TRANSITION -> elevator.isAtTarget();
            case EXECUTING_ACTION -> {
                if (action.endOnGamePieceNotSeen) {
                    yield !endEffector.hasCoral();
                } else if (action.endOnGamePieceSeen || action.toString().equals("INTAKE_FROM_HP_LOWER")) {
                    yield endEffector.hasCoral();
                } else {
                    yield false;
                }
            }
            case DONE -> true;
        };
    }

    public Optional<DoubleSupplier> getTargetWristRotationFraction(ScoringSuperstructureAction action) {
        return switch (this) {
            case TRANSITION_BEFORE_ELEVATOR,
                 ELEVATOR_MOVE_WITH_TRANSITION -> Optional.of(() -> WristSetpoints.Wrist_Transition_Proportion);
            case TRANSITION_AFTER_ELEVATOR,
                 ELEVATOR_MOVE_NO_TRANSITION,
                 EXECUTING_ACTION -> Optional.of(action.targetWristRotationFraction);
            case DONE -> Optional.empty();
        };
    }

    public Optional<DoubleSupplier> getTargetElevatorExtensionFraction(ScoringSuperstructureAction action) {
        return switch (this) {
            case TRANSITION_BEFORE_ELEVATOR -> Optional.empty();
            case ELEVATOR_MOVE_WITH_TRANSITION,
                 TRANSITION_AFTER_ELEVATOR,
                 ELEVATOR_MOVE_NO_TRANSITION,
                 EXECUTING_ACTION -> Optional.of(action.targetElevatorExtensionFraction);
            case DONE -> Optional.empty();
        };
    }

    public ScoringSuperstructureState next() {
        return switch (this) {
            case TRANSITION_BEFORE_ELEVATOR -> ELEVATOR_MOVE_WITH_TRANSITION;
            case ELEVATOR_MOVE_WITH_TRANSITION -> TRANSITION_AFTER_ELEVATOR;
            case TRANSITION_AFTER_ELEVATOR -> EXECUTING_ACTION;
            case ELEVATOR_MOVE_NO_TRANSITION -> EXECUTING_ACTION;
            case EXECUTING_ACTION -> DONE;
            case DONE -> DONE;
        };
    }

    public double getIntakeSpeed(ScoringSuperstructureAction action) {
        return switch (this) {
            case EXECUTING_ACTION -> action.intakeSpeed;
            default -> 0;
        };
    }
}
