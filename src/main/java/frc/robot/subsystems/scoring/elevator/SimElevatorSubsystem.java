package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Controls;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants.ProportionToHeight;
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
            ScoringSuperstructureAction.IDLE.getElevatorHeight().in(Meters),
            ScoringSuperstructureAction.SCORE_BARGE.getElevatorHeight().in(Meters),
            true,
            ScoringSuperstructureAction.IDLE.getElevatorHeight().in(Meters)
        );
    }

    @Override
    public double getCurrentExtensionFraction() {
        return ProportionToHeight.inverted().convert(Meters.of(
            elevatorSim.getPositionMeters()
        ));
    }

    @Override
    public void setTargetExtensionProportion(double targetExtensionProportion) {
        this.state = targetExtensionProportion;
        elevatorPID.setGoal(targetExtensionProportion.getElevatorAbsolutePosition());
    }

    @Override
    public void periodic() {
        if (isManualControlEnabled) {
            double outputVoltage = Controls.Operator.ManualControlElevator.getAsDouble() * 0.3;
            if (outputVoltage < 0) outputVoltage /= 2.;
            elevatorSim.setInputVoltage(outputVoltage * 12 + elevatorKg.get() * 3.8);
        }
        SmartDashboard.putBoolean("Is manual", isManualControlEnabled);

        updatePIDs();
        elevatorSim.update(0.020);
        elevatorSim.setState(elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void runElevator() {
        elevatorPID.setGoal(state.getElevatorAbsolutePosition());
        double output = elevatorPID.calculate(getCurrentPosition())
            + elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity);

        if (!isManualControlEnabled) {
            elevatorSim.setInputVoltage(output);
        }
        SmartDashboard.putNumber("Elevator PID output", output);
        SmartDashboard.putNumber("Elevator Sim Position", getCurrentPosition());
    }

    private void updatePIDs() {
        elevatorPID.setPID(
            ScoringPIDs.simElevatorKp.get(),
            ScoringPIDs.simElevatorKi.get(),
            ScoringPIDs.simElevatorKd.get()
        );
        elevatorPID.setConstraints(
            new TrapezoidProfile.Constraints(
                ScoringPIDs.simElevatorVelocity.get(),
                ScoringPIDs.simElevatorAcceleration.get()
            )
        );
    }

    @Override
    public void setRawMotorVoltage(Voltage voltage) {
        elevatorSim.setInputVoltage(voltage.in(Volts));
    }
}
