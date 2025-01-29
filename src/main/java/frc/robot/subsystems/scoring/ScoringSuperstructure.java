package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.scoring.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring.hopper.HopperSubsystem;

import java.util.Objects;

public class ScoringSuperstructure extends SubsystemBase {
    private static ScoringSuperstructure instance;

    public static ScoringSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
                ScoringSuperstructure::new
        );
    }

    private final ElevatorSubsystem elevator;
    private final HopperSubsystem hopper;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    public ScoringSuperstructure() {
        this.elevator = ElevatorSubsystem.getInstance();
        this.hopper = HopperSubsystem.getInstance(this);
    }

    private void setState(ScoringSuperstructureState state) {
        this.state = state;
        elevator.setElevatorState(state);
        hopper.setHopper(state);
    }

    // TODO: Don't append "Command" to command factory methods.
    public Command setScoringStateCommand(ScoringSuperstructureState state) {
        return Commands.runOnce(
                () -> setState(state),
                this
        );
    }

    public Command runScoringStateCommand() {
        return switch (state.firstToMove) {
            case BOTH -> Commands.run(
                    () -> {
                        elevator.runElevator();
                        hopper.runHopper();
                    },
                    this
            );
            case HOPPER -> Commands.run(
                    () -> {
                        hopper.runHopper();
                        if (hopper.atPositionStateTrigger().getAsBoolean()) {
                            elevator.runElevator();
                        }
                    },
                    this
            );
            case ELEVATOR -> Commands.run(
                    () -> {
                        elevator.runElevator();
                        if (elevator.isElevatorAtPosition()) {
                            hopper.runHopper();
                        }
                    },
                    this
            );
        };
    }

    // TODO: Remember, the trigger stuff here. Make basic getter methods
    public Trigger atPositionStateTrigger() {
        return elevator.isElevatorAtPosition
                .and(hopper.atPositionStateTrigger());
    }

    public Trigger stateFinishedTrigger() {
        return elevator.stateFinishedTrigger()
                .and(hopper.stateFinishedTrigger());
    }

    @Override
    public void periodic() {
        if (stateFinishedTrigger().getAsBoolean()) {
            setState(state.getStateAfter());
        } else if (!(state == ScoringSuperstructureState.IDLE) && !state.control.getAsBoolean()) {
            setState(ScoringSuperstructureState.IDLE);
        }
    }

    public Distance getCurrentElevatorLength() {
        return elevator.getCurrentLength();
    }

    public Distance getTargetElevatorLength() {
        return elevator.getTargetLength();
    }

    public Rotation2d getCurrentWristRotation() {
        return hopper.getCurrentRotation();
    }

    public Rotation2d getTargetWristRotation() {
        return hopper.getTargetRotation();
    }
}