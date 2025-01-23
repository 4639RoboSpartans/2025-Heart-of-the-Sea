package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;

    public static HopperSubsystem getInstance() {
        if (instance == null) {
            instance = new ConcreteHopperSubsystem();
        }
        return instance;
    }

    public abstract void setHopper(ScoringSuperstructureState state);

    public abstract void runHopperPosition();

    public abstract void runHopper();

    protected abstract boolean atPositionState();

    public abstract double getCurrentPosition();

    public Trigger atPositionStateTrigger() {
        return new Trigger(this::atPositionState);
    }

    public abstract double getTargetPosition();

    public abstract boolean isStateFinished();

    public Trigger stateFinishedTrigger() {
        return new Trigger(this::isStateFinished);
    }
}
