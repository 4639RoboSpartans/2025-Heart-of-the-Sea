package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import java.util.Objects;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;

    //Turns out, there is never more than one ScoringSuperstructure (good)
    //which means there is no reason to pass in the ScoringSuperstructure object
    //which means we do this instead
    public static HopperSubsystem getInstance() {
        //doing this to stop the hopper from existing before its ready without commenting out a bunch of code
        // (also, please multiline the comments instead of commenting out every line individually)
        //TODO: remove the false flag when hopper is ready
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(
                instance,
                ConcreteHopperSubsystem::new
            );
        } else {
            return instance = Objects.requireNonNullElseGet(
                instance,
                SimHopperSubsystem::new
            );
        }
    }

    protected ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    /**
     * Gets the current state of the hopper.
     * 
     * @return state of hopper as ScoringSuperStructureState
     */
    public final ScoringSuperstructureState getHopperState() {
        return state;
    }

    /**
     * Gets the current rotation of the wrist.
     * 
     * @return the current rotation of the wrist as Rotation2d
     */
    public abstract Rotation2d getCurrentRotation();

    /**
     * Gets the current rotation of the wrist by converting rotations to position.
     * 
     * @return position of wrist as double
     */
    public final double getCurrentPosition() {
        return ScoringConstants.HopperConstants.PositionToRotation.convertBackwards(getCurrentRotation());
    }

    /**
     * Gets the target rotation of the wrist.
     * 
     * @return target rotation of wrist as Rotation2d
     */
    public final Rotation2d getTargetRotation() {
        return state.getRotation();
    }

    /**
     * Gets the target rotation of the wrist by converting rotations to position.
     * 
     * @return target position of wrist as double
     */
    public final double getTargetPosition() {
        return ScoringConstants.HopperConstants.PositionToRotation.convertBackwards(getTargetRotation());
    }

    /**
     * Gets the speed of the rollers on the scoring mechanism.
     * 
     * @return speed of rollers as double
     */
    public abstract double getIntakeSpeed();

    /**
     * Checks if the wrist is at the target position.
     * 
     * @return if wrist at target position as boolean
     */
    public abstract boolean isAtTarget();

    /**
     * Checks if the wrist is at its desired state.
     * 
     * @return if wrist is at desired state as boolean
     */
    public abstract boolean isHopperStateFinished();

    /**
     * Checks if the scoring mechanism contains a coral.
     * 
     * @return if the scoring mechanism has a coral as boolean
     */
    public abstract boolean hasCoral();

    public Trigger hasCoral = new Trigger(this::hasCoral);

    /**
     * Sets the state of the hopper
     * 
     * @param state state that hopper is being set to as ScoringSuperstructureState.
     */
    public abstract void setHopper(ScoringSuperstructureState state);

    /**
     * Runs the wrist.
     */
    protected abstract void runHopperPosition();

    /**
     * Runs the intake/rollers.
     */
    public abstract void runHopper();

    protected boolean manualControlEnabled = false;

    public void setManualControlEnabled(boolean enabled) {
        manualControlEnabled = enabled;
    }
}
