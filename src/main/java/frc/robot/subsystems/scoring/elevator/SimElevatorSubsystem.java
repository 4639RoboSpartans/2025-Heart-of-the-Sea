package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.oi.OI;
import frc.robot.constants.Controls;
import frc.robot.subsystemManager.Subsystems;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.*;
import static frc.robot.subsystems.scoring.constants.ScoringPIDs.elevatorKg;

public class SimElevatorSubsystem extends AbstractElevatorSubsystem {
    private final ProfiledPIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final ElevatorSim elevatorSim;

    public SimElevatorSubsystem() {
        elevatorPID = new ProfiledPIDController(
            ScoringPIDs.simElevatorKp.get(),
            ScoringPIDs.simElevatorKi.get(),
            ScoringPIDs.simElevatorKd.get(),
            new TrapezoidProfile.Constraints(
                ScoringPIDs.simElevatorVelocity.get(),
                ScoringPIDs.simElevatorAcceleration.get()
            )
        );
        elevatorFeedforward = new ElevatorFeedforward(
            ScoringPIDs.simElevatorKs.get(),
            ScoringPIDs.simElevatorKg.get(),
            ScoringPIDs.simElevatorKv.get(),
            ScoringPIDs.simElevatorKa.get()
        );
        elevatorSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                10,
                0.0762,
                10
            ),
            DCMotor.getKrakenX60(2),
            STARTING_HEIGHT.in(Meters),
            MAX_HEIGHT.in(Meters) + STARTING_HEIGHT.in(Meters),
            true,
            STARTING_HEIGHT.in(Meters)
        );
    }

    @Override
    public ElevatorPosition getCurrentPosition() {
        return ElevatorPosition.fromHeight(Meters.of(
            elevatorSim.getPositionMeters()
        ));
    }

    @Override
    public void periodic() {
        if (Subsystems.scoringSuperstructure().isManualControlEnabled()) {
            double outputVoltage = Controls.Operator.ManualControlElevator.getAsDouble() * 0.3;
            if (outputVoltage < 0) outputVoltage /= 2.;
            elevatorSim.setInputVoltage(outputVoltage * 12 + elevatorKg.get() * 3.8);
        } else {
            elevatorPID.setGoal(getTargetPosition().getMotorPosition());
            double output = elevatorPID.calculate(getCurrentPosition().getMotorPosition())
                + elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity);

            elevatorSim.setInputVoltage(output);
        }

        elevatorSim.update(0.020);
        elevatorSim.setState(elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void setRawMotorVoltage(Voltage voltage) {
        elevatorSim.setInputVoltage(voltage.in(Volts));
    }

    @Override
    public boolean isPhysicallyStopped() {
        return elevatorSim.hasHitLowerLimit() || elevatorSim.hasHitUpperLimit()
            // For testing:
            || OI.getInstance().operatorController().XBOX_START_BUTTON.getAsBoolean();
    }

    @Override
    public void resetCurrentPositionTo(ElevatorPosition position) {
        elevatorSim.setState(
            position.getHeight().in(Meters),
            elevatorSim.getVelocityMetersPerSecond()
        );
        setTarget(position);
        elevatorPID.reset(position.getMotorPosition());
    }
}