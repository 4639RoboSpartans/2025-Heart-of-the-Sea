package frc.robot.subsystems.scoring.endeffector;

import com.revrobotics.spark.config.SparkBaseConfig;
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

    public static AbstractEndEffectorSubsystem getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);

        boolean dummy = false;
//        dummy = true;
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
    private double previousMotorPosition = 0;
    private double currentMotorVelocity = 0;

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

    public final double getCurrentMotorVelocity() {
        return currentMotorVelocity;
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

    public final Rotation2d getActionTargetRotation() {
        return ProportionToRotation.convert(getActionTargetRotationFraction());
    }

    /**
     * Gets the target rotation of the wrist by converting rotations to position.
     *
     * @return target position of wrist as double
     */
    public final double getTargetPosition() {
        return RotationFractionToMotorPosition.convert(getTargetRotationFraction());
    }

    public final double getActionTargetPosition() {
        return RotationFractionToMotorPosition.convert(getActionTargetRotationFraction());
    }

    public final double getTargetRotationFraction() {
        return targetRotationFraction;
    }

    public final double getActionTargetRotationFraction() {
        return SubsystemManager.getInstance().getScoringSuperstructure().getCurrentAction().targetWristRotationFraction;
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
        ) && getCurrentMotorVelocity() <= ScoringConstants.EndEffectorConstants.WRIST_VELOCITY_TOLERANCE;
    }

    public final boolean isWristAtActionTarget() {
        return MathUtil.isNear(
            getActionTargetPosition(),
            getCurrentMotorPosition(),
            WRIST_TOLERANCE
        ) && coralConsistentWithActionRequirement();
    }

    public final boolean coralConsistentWithActionRequirement() {
        var action = SubsystemManager.getInstance().getScoringSuperstructure().getCurrentAction();
        if (action.endOnGamePieceSeen) return hasCoral();
        if (action.endOnGamePieceNotSeen) return !hasCoral();
        return true;
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

        currentMotorVelocity = getCurrentMotorPosition() - previousMotorPosition;
        previousMotorPosition = getCurrentMotorPosition();

        if (SubsystemManager.getInstance().getScoringSuperstructure().isManualControlEnabled()) {
            targetWristRotationFraction = Controls.Operator.ManualControlWrist.getAsDouble();
            intakeSpeed = Controls.Operator.ManualControlIntake.getAsDouble();
        } else {
            targetWristRotationFraction = getTargetRotationFraction();
            intakeSpeed = this.intakeSpeed;
            if(Controls.Operator.ManualControlIntake.getAsDouble() != 0){
                intakeSpeed = Controls.Operator.ManualControlIntake.getAsDouble();
            }
        }

        SmartDashboard.putNumber("Intake Speed", intakeSpeed);

        periodic(targetWristRotationFraction, intakeSpeed);

        SmartDashboard.putString("Wrist info: ",
            "current fraction = " + currentWristRotationFraction
                + " target fraction = " + targetWristRotationFraction
        );
    }

    public abstract void setWristMotorIdleMode(SparkBaseConfig.IdleMode mode);

    public abstract boolean isWristPhysicallyStopped();

    public abstract void resetCurrentWristRotationFractionTo(double newWristRotationFraction);
}
