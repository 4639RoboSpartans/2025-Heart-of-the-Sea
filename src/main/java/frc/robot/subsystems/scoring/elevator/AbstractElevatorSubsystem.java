package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.Objects;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ProportionToHeight;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ProportionToPosition;

public abstract class AbstractElevatorSubsystem extends SubsystemBase {
    private static AbstractElevatorSubsystem instance;

    public static AbstractElevatorSubsystem getInstance() {
        boolean dummy = false;
        // dummy = true;
        if(dummy) return new DummyElevatorSubsystem();
        
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(instance, ConcreteElevatorSubsystem::new);
        } else {
            return instance = Objects.requireNonNullElseGet(instance, SimElevatorSubsystem::new);
        }
    }

    /**
     * The current state of the elevator
     */
    protected ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    protected boolean isManualControlEnabled = false;

    public void setManualControlEnabled(boolean manualControlEnabled) {
        isManualControlEnabled = manualControlEnabled;
    }

    /**
     * Get the current extension proportion of the elevator, where 0 means
     * fully retracted and 1 means fully extended
     */
    public abstract double getCurrentProportion();

    /**
     * Get the current position of the elevator. This is used by PID.
     */
    public double getCurrentPosition() {
        return ProportionToPosition.convert(getCurrentProportion());
    }

    /**
     * Get the current height of the elevator. This is used by simulation.
     */
    public Distance getCurrentHeight() {
        return ProportionToHeight.convert(getCurrentProportion());
    }

    /**
     * Get the target extension proportion of the elevator in the current state,
     * where 0 means fully retracted and 1 means fully extended
     */
    public final double getTargetProportion() {
        return state.elevatorProportion;
    }

    /**
     * Get the target position of the elevator in the current state. This is
     * used by PID
     */
    public final double getTargetPosition() {
        return ProportionToPosition.convert(getTargetProportion());
    }

    /**
     * Get the target height of the elevator in the current state. This is
     * used by simulation.
     */
    public final Distance getTargetHeight() {
        return ProportionToHeight.convert(getTargetProportion());
    }

    /**
     * Checks whether the elevator is at its target
     */
    public abstract boolean isAtTarget();

    // TODO: document this
    public abstract boolean isElevatorStateFinished();

    /**
     * Set the current state of the elevator
     */
    public abstract void updateElevatorState(ScoringSuperstructureState state);

    /**
     * Runs the elevator
     */
    public abstract void runElevator();

    /**
     * Set the raw output voltage of the motor. Used by SysID.
     */
    public abstract void setRawMotorVoltage(Voltage voltage);
}
