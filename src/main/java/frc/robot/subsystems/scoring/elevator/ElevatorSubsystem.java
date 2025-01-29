package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.robot.Robot;
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

    // TODO: unify "position" and "length" if possible. Why does "position" mean different things in two classes?
    public abstract double getCurrentPosition();

    public abstract Distance getCurrentLength();

    public abstract double getTargetPosition();

    public abstract Distance getTargetLength();

    // TODO: implement this pattern everywhere else there is a trigger
    public abstract boolean isElevatorAtPosition();
    public final Trigger isElevatorAtPosition = new Trigger(this::isElevatorAtPosition);

    public abstract boolean isElevatorStateFinished();

    public Trigger stateFinishedTrigger() {
        return new Trigger(this::isElevatorStateFinished);
    }

    public abstract void setElevatorState(ScoringSuperstructureState state);

    public abstract void runElevator();

    // Todo: PLEASE MOVE SYS ID TO SEPARATE CLASSES EVERYWHERE!

    public abstract Command quasistatic(SysIdRoutine.Direction direction);

    public abstract Command dynamic(SysIdRoutine.Direction direction);
}
