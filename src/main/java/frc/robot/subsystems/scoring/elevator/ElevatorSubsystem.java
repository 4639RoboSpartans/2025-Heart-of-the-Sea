package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.Objects;

public abstract class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    public static ElevatorSubsystem getInstance() {
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(instance, ConcreteElevatorSubsystem::new);
        } else {
            return instance = Objects.requireNonNullElseGet(instance, SimElevatorSubsystem::new);
        }
    }

    public abstract void setElevatorState(ScoringSuperstructureState state);

    public abstract void runElevator();

    public abstract boolean isElevatorAtPositionState();

    public abstract double getCurrentPosition();

    public Trigger atPositionStateTrigger() {
        return new Trigger(this::isElevatorAtPositionState);
    }

    public abstract double getTargetPosition();

    public abstract boolean isElevatorStateFinished();

    public Trigger stateFinishedTrigger() {
        return new Trigger(this::isElevatorStateFinished);
    }

    public abstract Command quasistatic(SysIdRoutine.Direction direction);

    public abstract Command dynamic(SysIdRoutine.Direction direction);
}
