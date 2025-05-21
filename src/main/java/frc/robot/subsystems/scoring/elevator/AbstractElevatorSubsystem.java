package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;

import java.util.Objects;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.*;

public abstract class AbstractElevatorSubsystem extends SubsystemBase {
    private static AbstractElevatorSubsystem instance;

    public static AbstractElevatorSubsystem getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);

        boolean dummy = false;
        //dummy = true;
        if (dummy) return new DummyElevatorSubsystem();

        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(instance, ConcreteElevatorSubsystem::new);
        } else {
            return instance = Objects.requireNonNullElseGet(instance, SimElevatorSubsystem::new);
        }
    }

    private double targetExtensionProportion = ScoringSuperstructureAction.IDLE.targetElevatorExtensionFraction.getAsDouble();

    protected boolean isManualControlEnabled = false;

    public void setManualControlEnabled(boolean manualControlEnabled) {
        isManualControlEnabled = manualControlEnabled;
    }

    /**
     * Get the current extension proportion of the elevator, where 0 means
     * fully retracted and 1 means fully extended
     */
    public abstract double getCurrentExtensionFraction();

    /**
     * Get the current position of the elevator. This is used by PID.
     */
    public double getCurrentPosition() {
        return ProportionToPosition.convert(getCurrentExtensionFraction());
    }

    /**
     * Get the current height of the elevator. This is used by simulation.
     */
    public Distance getCurrentHeight() {
        return ProportionToHeight.convert(getCurrentExtensionFraction());
    }

    /**
     * Get the target extension proportion of the elevator in the current state,
     * where 0 means fully retracted and 1 means fully extended
     */
    public final double getTargetExtensionFraction() {
        return targetExtensionProportion;
    }

    /**
     * Get the target position of the elevator in the current state. This is
     * used by PID
     */
    public final double getTargetPosition() {
        return ProportionToPosition.convert(getTargetExtensionFraction());
    }

    /**
     * Get the target height of the elevator in the current state. This is
     * used by simulation.
     */
    public final Distance getTargetHeight() {
        return ProportionToHeight.convert(getTargetExtensionFraction());
    }

    public boolean isAtTarget() {
        return MathUtil.isNear(
            getCurrentPosition(),
            getTargetPosition(),
            ELEVATOR_TOLERANCE
        );
    }

    public boolean isNearTarget() {
        return MathUtil.isNear(
            getCurrentPosition(),
            getTargetPosition(),
            ELEVATOR_TOLERANCE * 25
        );
    }

    public abstract boolean isDangerouslyStopped();

    /**
     * Set the target extension proportion of the elevator in the current state,
     * where 0 means fully retracted and 1 means fully extended
     */
    public final void setTargetExtensionFraction(double targetExtensionProportion) {
        this.targetExtensionProportion = targetExtensionProportion;
    }

    /**
     * Set the raw output voltage of the motor. Used by SysID.
     */
    public abstract void setRawMotorVoltage(Voltage voltage);

    public abstract boolean isPhysicallyStopped();

    public abstract void resetCurrentExtensionFractionTo(double extensionFraction);
}
