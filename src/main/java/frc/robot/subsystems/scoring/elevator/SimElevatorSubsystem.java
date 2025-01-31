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

    private boolean isStateFinished = false;

    public SimElevatorSubsystem() {
        elevatorPID = new ProfiledPIDController(
                ScoringPIDs.elevatorKp.get(),
                ScoringPIDs.elevatorKi.get(),
                ScoringPIDs.elevatorKd.get(),
                new TrapezoidProfile.Constraints(
                        ScoringPIDs.elevatorVelocity.get(),
                        ScoringPIDs.elevatorAcceleration.get()
                )
        );
        elevatorFeedforward = new ElevatorFeedforward(
                ScoringPIDs.elevatorKs.get(),
                ScoringPIDs.elevatorKg.get(),
                ScoringPIDs.elevatorKv.get(),
                ScoringPIDs.elevatorKa.get()
        );
        elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        DCMotor.getKrakenX60(2),
                        10,
                        0.0762,
                        10
                ),
                DCMotor.getKrakenX60(2),
                ScoringSuperstructureState.IDLE.getElevatorLength().in(Meters),
                ScoringSuperstructureState.BARGE_SCORING.getElevatorLength().in(Meters),
                true,
                ScoringSuperstructureState.IDLE.getElevatorLength().in(Meters)
        );
    }

    @Override
    public double getCurrentPosition() {
        return ScoringSuperstructureState.getElevatorSimPosition(getCurrentLength());
    }

    @Override
    public Distance getCurrentLength() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    @Override
    public double getTargetPosition() {
        return ScoringSuperstructureState.getElevatorSimPosition(getTargetLength());
    }

    @Override
    public Distance getTargetLength() {
        return state.getElevatorLength();
    }

    public boolean isElevatorStateFinished() {
        return isStateFinished;
    }

    @Override
    public boolean isElevatorAtPosition() {
        return MathUtil.isNear(
                ScoringSuperstructureState.getElevatorSimPosition(getTargetLength()),
                ScoringSuperstructureState.getElevatorSimPosition(getCurrentLength()),
                ScoringConstants.ElevatorConstants.ELEVATOR_TOLERANCE
        ) && elevatorPID.getVelocityError() <= 0.01;
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

        if (isElevatorAtPosition()) {
            isStateFinished = true;
        }
    }

    @Override
    public void runElevator() {
        elevatorPID.setGoal(ScoringSuperstructureState.getElevatorSimPosition(state.getElevatorLength()));
        double output = -elevatorPID.calculate(getCurrentPosition())
                + elevatorFeedforward.calculate(getCurrentPosition());
        elevatorSim.setInputVoltage(output);
        SmartDashboard.putNumber("Elevator PID output", output);
    }

    private void updatePIDs() {
        elevatorPID.setPID(
                ScoringPIDs.elevatorKp.get(),
                ScoringPIDs.elevatorKi.get(),
                ScoringPIDs.elevatorKd.get()
        );
        elevatorPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ScoringPIDs.elevatorVelocity.get(),
                        ScoringPIDs.elevatorAcceleration.get()
                )
        );
    }

    @Override
    public void setElevatorMotorVoltsSysID(Voltage voltage) {
        elevatorSim.setInputVoltage(voltage.in(Volts));
    }
}
