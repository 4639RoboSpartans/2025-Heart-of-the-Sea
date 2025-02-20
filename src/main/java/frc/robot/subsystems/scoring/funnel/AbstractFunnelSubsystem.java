package frc.robot.subsystems.scoring.funnel;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;

public abstract class AbstractFunnelSubsystem extends SubsystemBase{
    private static AbstractFunnelSubsystem instance;

    public static AbstractFunnelSubsystem getInstance() {
        boolean dummy = false;
        // dummy = true;
        if(dummy) return new DummyFunnelSubsystem();
        
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(instance, ConcreteFunnelSubsystem::new);
        } else {
            return instance = Objects.requireNonNullElseGet(instance, DummyFunnelSubsystem::new);
        }
    }

    protected boolean isDown = true;
    protected boolean isTargetPositionDown = true;

    /**
     * Checks if the funnel is at its desired state.
     * 
     * @return if funnel is at desired state as boolean
     */
    public boolean isFunnelStateFinished(){
        return isDown != isTargetPositionDown;
    }

    protected boolean isManualControlEnabled = false;

    /**
     * Sets the funnel to manual control
     * 
     * @Param if manual control is enabled
     */
    public void setManualControlEnabled(boolean enabled) {
        isManualControlEnabled = enabled;
    }

    public void setDown(boolean toSetDown){
        isTargetPositionDown = toSetDown;
    }

    public abstract double getCurrent();
}
