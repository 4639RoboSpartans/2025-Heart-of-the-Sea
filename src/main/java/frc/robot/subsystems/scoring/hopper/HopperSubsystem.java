package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;

    public static HopperSubsystem getInstance(ScoringSuperstructure scoringSuperstructure) {
        if (instance == null) {
            instance = new ConcreteHopperSubsystem(scoringSuperstructure);
        }
        return instance;
    }

    public abstract void setHopper(ScoringSuperstructureState state);

    protected abstract void runHopperPosition();

    public abstract void runHopper();

    protected abstract boolean isHopperAtPositionState();

    public abstract double getCurrentPosition();

    public Trigger atPositionStateTrigger() {
        return new Trigger(this::isHopperAtPositionState);
    }

    public abstract double getTargetPosition();

    public abstract boolean isHopperStateFinished();

    public Trigger stateFinishedTrigger() {
        return new Trigger(this::isHopperStateFinished);
    }
}
