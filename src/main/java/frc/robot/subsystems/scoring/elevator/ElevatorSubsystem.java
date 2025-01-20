package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public abstract class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            instance = new ConcreteElevatorSubsystem();
        }
        return instance;
    }

    public abstract void setElevatorState(ScoringSuperstructureState state);

    public abstract void runElevator();

    protected abstract boolean atState();

    public abstract double getCurrentPosition();

    public Trigger atRequestedStateTrigger() {
        return new Trigger(this::atState);
    }

    public abstract double getTargetPosition();

    public abstract boolean isStateFinished();

    public abstract Command quasistatic(SysIdRoutine.Direction direction);

    public abstract Command dynamic(SysIdRoutine.Direction direction);
}
