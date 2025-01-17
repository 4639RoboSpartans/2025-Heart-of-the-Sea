package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.scoring.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoring.gripper.GripperSubsystem;

import java.util.Objects;

public class ScoringSuperstructure extends SubsystemBase {
    private static ScoringSuperstructure instance;

    public static ScoringSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
                () -> new ScoringSuperstructure(
                        ElevatorSubsystem.getInstance(),
                        GripperSubsystem.getInstance()
                )
        );
    }

    private final ElevatorSubsystem elevator;
    private final GripperSubsystem gripper;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    public ScoringSuperstructure(ElevatorSubsystem elevator, GripperSubsystem gripper) {
        this.elevator = elevator;
        this.gripper = gripper;
    }

    public void setState(ScoringSuperstructureState state) {
        this.state = state;
        elevator.setElevatorState(state);
        gripper.setGripperState(state);
    }

    public Trigger atStateTrigger() {
        return elevator.atRequestedStateTrigger()
                .and(gripper.atRequestedStateTrigger());
    }

    @Override
    public void periodic() {
        if (state.isStateFinished()){
            switch (state){
                case L1, L2, L3, L4:
                    setState(ScoringSuperstructureState.HP_LOADING);
                    break;
                case HANDOFF:
                    setState(ScoringSuperstructureState.IDLE);
                    break;
            }
        }
    }
}