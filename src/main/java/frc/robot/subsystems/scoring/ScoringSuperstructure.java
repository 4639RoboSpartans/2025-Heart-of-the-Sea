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

    public Command setScoringState(ScoringSuperstructureState state) {
        return Commands.runOnce(
                () -> setState(state),
                this
        );
    }

    public Command runScoringState() {
        if (state.lastToMove == null) {
            return Commands.run(
                    () -> {
                        hopper.runHopper();
                        elevator.runElevator();
                    },
                    this
            );
        } else if (state.lastToMove == ElevatorSubsystem.class) {
            return Commands.run(
                    () -> {
                        hopper.runHopper();
                        if (hopper.isHopperAtPosition()) {
                            elevator.runElevator();
                        }
                    },
                    this
            );
        } else if (state.lastToMove == HopperSubsystem.class) {
            return Commands.run(
                    () -> {
                        elevator.runElevator();
                        if (elevator.isElevatorAtPosition()) {
                            hopper.runHopper();
                        }
                    },
                    this
            );
        }
        return null;
    }

    public boolean isAtPositionState() {
        return elevator.isElevatorAtPosition() && hopper.isHopperAtPosition();
    }
    public Trigger isAtPositionState = new Trigger(this::isAtPositionState);

    public boolean isStateFinished() {
        return elevator.isElevatorStateFinished() && hopper.isHopperStateFinished();
    }
    public Trigger isStateFinished = new Trigger(this::isStateFinished);

    @Override
    public void periodic() {
        if (isStateFinished()) {
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