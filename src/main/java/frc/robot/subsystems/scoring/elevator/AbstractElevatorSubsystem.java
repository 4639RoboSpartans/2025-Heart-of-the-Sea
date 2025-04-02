package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.units.Measurement;
import frc.robot.subsystemManager.SubsystemInstantiator;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.*;

public abstract class AbstractElevatorSubsystem extends SubsystemBase {
    public static SubsystemInstantiator<AbstractElevatorSubsystem> getInstantiator() {
        return new SubsystemInstantiator<>(
            ConcreteElevatorSubsystem::new,
            SimElevatorSubsystem::new,
            DummyElevatorSubsystem::new,
            false
        );
    }

    private ElevatorPosition targetPosition =
        ScoringSuperstructureAction.IDLE.targetElevatorPosition.get();

    /**
     * Get the current position of the elevator
     */
    public abstract ElevatorPosition getCurrentPosition();

    /**
     * Get the target position of the elevator
     */
    public final ElevatorPosition getTargetPosition() {
        return targetPosition;
    }

    /**
     * Set the target position of the elevator
     */
    public final void setTarget(ElevatorPosition targetPosition) {
        this.targetPosition = targetPosition;
    }

    public boolean isAtTarget() {
        return Measurement.withTolerance(
            getTargetPosition(),
            ELEVATOR_AT_TARGET_TOLERANCE
        ).contains(getCurrentPosition());
    }

    public boolean isNearTarget() {
        return Measurement.withTolerance(
            getTargetPosition(),
            ELEVATOR_NEAR_TARGET_TOLERANCE
        ).contains(getCurrentPosition());
    }

    /**
     * Set the raw output voltage of the motor. Used by SysID.
     */
    public abstract void setRawMotorVoltage(Voltage voltage);

    public abstract boolean isPhysicallyStopped();

    public abstract void resetCurrentPositionTo(ElevatorPosition position);
}
