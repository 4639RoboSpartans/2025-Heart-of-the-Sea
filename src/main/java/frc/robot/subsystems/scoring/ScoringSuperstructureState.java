package frc.robot.subsystems.scoring;

import frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.WristSetpoints;
import frc.robot.subsystems.scoring.elevator.AbstractElevatorSubsystem;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

import java.util.OptionalDouble;

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
            case TRANSITION_BEFORE_ELEVATOR -> endEffector.isWristAtTarget();
            case ELEVATOR_MOVE_WITH_TRANSITION -> elevator.isAtTarget();
            case TRANSITION_AFTER_ELEVATOR -> endEffector.isWristAtTarget();
            case ELEVATOR_MOVE_NO_TRANSITION -> elevator.isAtTarget();
            case EXECUTING_ACTION -> {
                if (action.endOnGamePieceNotSeen) {
                    yield !endEffector.hasCoral();
                } else if (action.endOnGamePieceSeen || action.toString().equals("INTAKE_FROM_HP")) {
                    yield endEffector.hasCoral();
                } else {
                    yield false;
                }
            }
            case DONE -> true;
        };
    }

    public OptionalDouble getTargetWristRotationFraction(ScoringSuperstructureAction action) {
        return switch (this) {
            case TRANSITION_BEFORE_ELEVATOR,
                 ELEVATOR_MOVE_WITH_TRANSITION -> OptionalDouble.of(WristSetpoints.Wrist_Transition_Proportion);
            case TRANSITION_AFTER_ELEVATOR,
                 ELEVATOR_MOVE_NO_TRANSITION,
                 EXECUTING_ACTION -> OptionalDouble.of(action.targetWristRotationFraction);
            case DONE -> OptionalDouble.empty();
        };
    }

    public OptionalDouble getTargetElevatorExtensionFraction(ScoringSuperstructureAction action) {
        return switch (this) {
            case TRANSITION_BEFORE_ELEVATOR -> OptionalDouble.empty();
            case ELEVATOR_MOVE_WITH_TRANSITION,
                 TRANSITION_AFTER_ELEVATOR,
                 ELEVATOR_MOVE_NO_TRANSITION,
                 EXECUTING_ACTION -> OptionalDouble.of(action.targetElevatorExtensionFraction);
            case DONE -> OptionalDouble.empty();
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
        return Math.abs(switch (this) {
            case EXECUTING_ACTION -> action.intakeSpeed ;
            default -> 0;
        });
    }
}
