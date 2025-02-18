package frc.robot.subsystems.scoring.endeffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.Objects;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.PositionToRotation;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.ProportionToRotation;

public abstract class AbstractEndEffectorSubsystem extends SubsystemBase {
    private static AbstractEndEffectorSubsystem instance;

    public static AbstractEndEffectorSubsystem getInstance() {
        boolean dummy = false;
        // dummy = true
        if (dummy) return new DummyEndEffectorSubsystem();
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(
                instance,
                ConcreteEndEffectorSubsystem::new
            );
        } else {
            return instance = Objects.requireNonNullElseGet(
                instance,
                SimEndEffectorSubsystem::new
            );
        }
    }

    protected ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    /**
     * Gets the current state of the endeffector.
     *
     * @return state of endeffector as ScoringSuperStructureState
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
        return PositionToRotation.convertBackwards(getCurrentRotation());
    }

    /**
     * Gets the target rotation of the wrist.
     *
     * @return target rotation of wrist as Rotation2d
     */
    public final Rotation2d getTargetRotation() {
        return ProportionToRotation.convert(state.targetWristRotationFraction);
    }

    /**
     * Gets the target rotation of the wrist by converting rotations to position.
     *
     * @return target position of wrist as double
     */
    public final double getTargetPosition() {
        return PositionToRotation.convertBackwards(getTargetRotation());
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
     * Sets the state of the endeffector
     *
     * @param state state that endeffector is being set to as ScoringSuperstructureState.
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
