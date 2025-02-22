package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystems.SubsystemManager;

import java.util.Objects;

public abstract class AbstractFunnelSubsystem extends SubsystemBase {
    private static AbstractFunnelSubsystem instance;

    public static AbstractFunnelSubsystem getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);

        boolean dummy = false;
        // dummy = true;
        if(dummy) return new DummyFunnelSubsystem();
        
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(instance, ConcreteFunnelSubsystem::new);
        } else {
            return instance = Objects.requireNonNullElseGet(instance, DummyFunnelSubsystem::new);
        }
    }

    /**
     * Checks if the funnel is at its desired state.
     * 
     * @return if funnel is at desired state as boolean
     */
    public boolean isFunnelStateFinished(){
        return active != isTargetPositionDown;
    }

    protected boolean isTargetPositionDown = true;
    protected boolean active = false;

    /**
     * Gets if the funnel is active/down 
     * 
     * @return boolean if funnel is active/down
     */
    public boolean isActive(){
        return active;
    }

    /**
     * Sets whether the funnel is active
     *
     * @param active Whether the funnel is in intaking position
     */
    public final void setFunnelActive(boolean isTargetPositionDown) {
        this.isTargetPositionDown = isTargetPositionDown;
    }

    /**
     * Gets the current of the pivot motor
     * 
     * @return current of pivot motor as double
     */
    public abstract double getCurrent();

    protected boolean isManualControlEnabled = false;

    /**
     * Sets the funnel to manual control
     * 
     * @Param if manual control is enabled
     */
    public void setManualControlEnabled(boolean enabled) {
        isManualControlEnabled = enabled;
    }
}
