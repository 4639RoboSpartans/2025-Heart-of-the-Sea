package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import static edu.wpi.first.units.Units.Meters;

public class SimElevatorSubsystem extends ElevatorSubsystem {
    private final ProfiledPIDController elevatorPID;
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
        elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        DCMotor.getKrakenX60(2),
                        2,
                        1,
                        10
                ),
                DCMotor.getKrakenX60(2),
                ScoringSuperstructureState.IDLE.getElevatorSimLength().in(Meters),
                ScoringSuperstructureState.L4.getElevatorSimLength().in(Meters),
                true,
                ScoringSuperstructureState.IDLE.getElevatorSimLength().in(Meters)
        );
    }

    @Override
    public double getCurrentPosition() {
        return elevatorSim.getPositionMeters();
    }

    private Distance getCurrentSimDistance() {
        return Meters.of(getCurrentPosition());
    }

    @Override
    public double getTargetPosition() {
        return state.getElevatorSimLength().in(Meters);
    }

    private Distance getTargetSimDistance() {
        return Meters.of(getTargetPosition());
    }

    public boolean isElevatorStateFinished() {
        return isStateFinished;
    }

    @Override
    public boolean isElevatorAtPositionState() {
        return MathUtil.isNear(
                ScoringSuperstructureState.getElevatorSimPosition(getTargetSimDistance()),
                ScoringSuperstructureState.getElevatorSimPosition(getCurrentSimDistance()),
                ScoringConstants.ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    @Override
    public void setElevatorState(ScoringSuperstructureState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        updatePIDs();
        elevatorSim.update(0.020);
        elevatorSim.setState(elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());

        if (isElevatorAtPositionState()) {
            isStateFinished = true;
        }
    }

    @Override
    public void runElevator() {
        elevatorPID.setGoal(ScoringSuperstructureState.getElevatorSimPosition(state.getElevatorSimLength()));
        double output = elevatorPID.calculate(
                ScoringSuperstructureState.getElevatorSimPosition(
                        getCurrentSimDistance()
                )
        );
        elevatorSim.setInput(output);
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

    public Command quasistatic(SysIdRoutine.Direction direction) {
        return new WaitCommand(0.5);
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return quasistatic(direction);
    }
}
