package frc.robot.subsystems.scoring.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.gripper.ConcreteGripperSubsystem;

public abstract class GripperSubsystem extends SubsystemBase {
    private static GripperSubsystem instance;

    public static GripperSubsystem getInstance() {
        if (instance == null) {
            instance = new ConcreteGripperSubsystem();
        }
        return instance;
    }

    public abstract void setGripperState(ScoringSuperstructureState state);

    public abstract void runGripper();

    protected abstract boolean atState();

    public abstract double getCurrentPosition();

    public Trigger atRequestedStateTrigger() {
        return new Trigger(this::atState);
    }

    public abstract double getTargetPosition();

    public abstract Command quasistatic(SysIdRoutine.Direction direction);

    public abstract Command dynamic(SysIdRoutine.Direction direction);
}
