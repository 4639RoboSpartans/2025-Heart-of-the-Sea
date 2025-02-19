package frc.robot.subsystems.scoring.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot.Robot;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import java.util.Objects;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.*;

public abstract class AbstractEndEffectorSubsystem extends SubsystemBase {
    private static AbstractEndEffectorSubsystem instance;
    protected double intakeSpeed;

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

    protected double targetRotationFraction;

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

    public double getCurrentRotationFraction() {
        return ProportionToRotation.convertBackwards(getCurrentRotation());
    }

    /**
     * Gets the target rotation of the wrist.
     *
     * @return target rotation of wrist as Rotation2d
     */
    public final Rotation2d getTargetRotation() {
        return ProportionToRotation.convert(getTargetRotationFraction());
    }

    /**
     * Gets the target rotation of the wrist by converting rotations to position.
     *
     * @return target position of wrist as double
     */
    public final double getTargetPosition() {
        return ProportionToPosition.convert(getTargetRotationFraction());
    }

    public final double getTargetRotationFraction() {
        return targetRotationFraction;
    }

    public final void setTargetRotationFraction(double targetRotationFraction) {
        this.targetRotationFraction = targetRotationFraction;
    }

    public final void setIntakeSpeed(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
    }

    public final boolean isWristAtTarget() {
        return MathUtil.isNear(
            getTargetPosition(),
            getCurrentPosition(),
            ScoringConstants.EndEffectorConstants.WRIST_TOLERANCE
        );
    }

    /**
     * Checks if the scoring mechanism contains a coral.
     *
     * @return if the scoring mechanism has a coral as boolean
     */
    public abstract boolean hasCoral();

    public Trigger hasCoral = new Trigger(this::hasCoral);

    protected boolean manualControlEnabled = false;

    public void setManualControlEnabled(boolean enabled) {
        manualControlEnabled = enabled;
    }
}
