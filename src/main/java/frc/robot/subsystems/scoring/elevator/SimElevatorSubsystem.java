package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

public class SimElevatorSubsystem extends ElevatorSubsystem {
    private final ProfiledPIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final ElevatorSim elevatorSim;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

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
                ScoringSuperstructureState.IDLE.getElevatorHeight().in(Meters),
                ScoringSuperstructureState.BARGE_SCORING.getElevatorHeight().in(Meters),
                true,
                ScoringSuperstructureState.IDLE.getElevatorHeight().in(Meters)
        );
    }

    @Override
    public double getCurrentPosition() {
        return ScoringConstants.ElevatorConstants.PositionToHeight.inverted().convert(getCurrentLength());
    }

    @Override
    public Distance getCurrentLength() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    @Override
    public double getTargetPosition() {
        return state.getElevatorAbsolutePosition();
    }

    @Override
    public Distance getTargetLength() {
        return state.getElevatorHeight();
    }

    public boolean isElevatorStateFinished() {
        return isElevatorAtPosition();
    }

    @Override
    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(
                ScoringConstants.ElevatorConstants.PositionToHeight.inverted().convert(getTargetLength()),
                ScoringConstants.ElevatorConstants.PositionToHeight.inverted().convert(getCurrentLength()),
                ScoringConstants.ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    @Override
    public void setElevatorState(ScoringSuperstructureState state) {
        this.state = state;
        elevatorPID.setGoal(state.getElevatorAbsolutePosition());
    }

    @Override
    public void periodic() {
        updatePIDs();
        elevatorSim.update(0.020);
        elevatorSim.setState(elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void runElevator() {
        elevatorPID.setGoal(state.getElevatorAbsolutePosition());
        double output = elevatorPID.calculate(getCurrentPosition())
                + elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity);
        elevatorSim.setInputVoltage(output);
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
    public void setElevatorMotorVoltsSysID(Voltage voltage) {
        elevatorSim.setInputVoltage(voltage.in(Volts));
    }
}
