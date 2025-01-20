package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.scoring.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring.gripper.HopperSubsystem;

import java.util.Objects;

public class ScoringSuperstructure extends SubsystemBase {
    private static ScoringSuperstructure instance;

    public static ScoringSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
                () -> new ScoringSuperstructure(
                        ElevatorSubsystem.getInstance(),
                        HopperSubsystem.getInstance()
                )
        );
    }

    private final ElevatorSubsystem elevator;
    private final HopperSubsystem hopper;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    public ScoringSuperstructure(ElevatorSubsystem elevator, HopperSubsystem hopper) {
        this.elevator = elevator;
        this.hopper = hopper;
    }

    public void setState(ScoringSuperstructureState state) {
        this.state = state;
        elevator.setElevatorState(state);
        hopper.setHopper(state);
    }

    public Trigger atStateTrigger() {
        return elevator.atRequestedStateTrigger()
                .and(hopper.atRequestedStateTrigger());
    }

    @Override
    public void periodic() {
        if (hopper.isStateFinished() && elevator.isStateFinished()){
            setState(state.getStateAfter());
        }
    }
}