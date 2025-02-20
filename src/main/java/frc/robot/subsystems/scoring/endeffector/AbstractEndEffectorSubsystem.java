package frc.robot.subsystems.scoring.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Controls;
import frc.robot.robot.Robot;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import java.util.Objects;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.EndEffectorConstants.*;

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

    private double intakeSpeed = 0;
    private double targetRotationFraction = 0;

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
    public final double getCurrentMotorPosition() {
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
        return RotationFractionToMotorPosition.convert(getTargetRotationFraction());
    }

    public final double getTargetRotationFraction() {
        return targetRotationFraction;
    }

    public final void setTargetWristRotationFraction(double targetRotationFraction) {
        this.targetRotationFraction = targetRotationFraction;
    }

    public final void setIntakeSpeed(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
    }

    public final boolean isWristAtTarget() {
        return MathUtil.isNear(
            getTargetPosition(),
            getCurrentMotorPosition(),
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

    protected abstract void periodic(double targetWristRotationFraction, double intakeSpeed);

    @Override
    public final void periodic() {
        double currentWristRotationFraction = getCurrentRotationFraction();
        double targetWristRotationFraction;
        double intakeSpeed;

        if (SubsystemManager.getInstance().getScoringSuperstructure().isManualControlEnabled()) {
            targetWristRotationFraction = Controls.Operator.ManualControlWrist.getAsDouble();
            intakeSpeed = Controls.Operator.ManualControlIntake.getAsDouble();
        } else {
            targetWristRotationFraction = getTargetRotationFraction();
            intakeSpeed = this.intakeSpeed;
        }

        periodic(targetWristRotationFraction, intakeSpeed);

        SmartDashboard.putString("Wrist info: ",
            "current fraction = " + currentWristRotationFraction
                + " target fraction = " + targetWristRotationFraction
        );
    }
}
