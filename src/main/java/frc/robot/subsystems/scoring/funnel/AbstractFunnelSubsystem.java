package frc.robot.subsystems.scoring.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public abstract class AbstractFunnelSubsystem extends SubsystemBase{
    private static AbstractFunnelSubsystem instance;

    protected ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    /**
     * Gets the current state of the funnel.
     * 
     * @return state of funnel as ScoringSuperStructureState
     */
    public final ScoringSuperstructureState getFunnelState() {
        return state;
    }

    /**
     * Checks if the funnel is at its desired state.
     * 
     * @return if funnel is at desired state as boolean
     */
    public abstract boolean isFunnelStateFinished();

    /**
     * Sets the state of the funnel
     * 
     * @param state state that funnel is being set to as ScoringSuperstructureState.
     */
    public abstract void setFunnel(ScoringSuperstructureState state);

    /**
     * Runs the funnel.
     */
    protected abstract void runFunnelPosition();

    protected boolean manualControlEnabled = false;

    /**
     * Sets the funnel to manual control
     * 
     * @Param if manual control is enabled
     */
    public void setManualControlEnabled(boolean enabled) {
        manualControlEnabled = enabled;
    }
}
