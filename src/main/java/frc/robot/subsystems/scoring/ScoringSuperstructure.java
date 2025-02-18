package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.annotation.ForSubsystemManagerUseOnly;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.elevator.AbstractElevatorSubsystem;
import frc.robot.subsystems.scoring.elevator.ElevatorSysID;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

import java.util.Objects;
import java.util.function.Supplier;

public class ScoringSuperstructure extends SubsystemBase {
    private static ScoringSuperstructure instance;

    /**
     * This method should only be accessed from the SubsystemManager class. In other places, use
     * {@link SubsystemManager#getScoringSuperstructure()} instead.
     */
    @ForSubsystemManagerUseOnly
    public static ScoringSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
            ScoringSuperstructure::new
        );
    }

    private final AbstractElevatorSubsystem elevator;
    private final AbstractEndEffectorSubsystem hopper;

    private ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;
    private ScoringSuperstructureState prevState = ScoringSuperstructureState.IDLE;
    private boolean isManualControlEnabled = false;

    public ScoringSuperstructure() {
        this.elevator = AbstractElevatorSubsystem.getInstance();
        this.hopper = AbstractEndEffectorSubsystem.getInstance();
        SmartDashboard.putBoolean("isManualControlEnabled", isManualControlEnabled);
    }

    private void setState(ScoringSuperstructureState state) {
        if (this.state != state) {
            prevState = this.state;
        }
        this.state = state;
        elevator.updateElevatorState(state);
        hopper.setHopper(state);
    }

    /**
     * @param state the new state to set the scoring superstructure to.
     * @return an instantaneous command that sets the state to {@param state}. It does not run the scoring
     * superstructure, only sets the state.
     */
    public Command setScoringState(ScoringSuperstructureState state) {
        return setScoringState(() -> state);
    }

    /**
     * @param state the new state to set the scoring superstructure to.
     * @return an instantaneous command that sets the state to {@param state}. It does not run the scoring
     * superstructure, only sets the state.
     */
    public Command setScoringState(Supplier<ScoringSuperstructureState> state) {
        return Commands.runOnce(
            () -> setState(state.get())
        );
    }

    public Command hold() {
        return setScoringState(() -> ScoringSuperstructureState.HOLD(
            elevator.getCurrentProportion(),
            ScoringConstants.EndEffectorConstants.ProportionToPosition.convertBackwards(hopper.getCurrentPosition())
        ));
    }

    /**
     * @return command to run the scoring subsystem. It takes into account which "sub-subsystem" is the last to move.
     */
    public Command runScoringState() {
        return Commands.run(
            () -> {
                if (!state.useTransitionState || !prevState.useTransitionState) {
                    elevator.runElevator();
                } else {
                    if (hopper.getHopperState() != ScoringSuperstructureState.TRANSITION_STATE) {
                        if (elevator.isAtTarget()) {
                            elevator.runElevator();
                        } else {
                            hopper.setHopper(ScoringSuperstructureState.TRANSITION_STATE);
                        }
                    } else {
                        if (hopper.isAtTarget()) {
                            if (elevator.isAtTarget()) {
                                hopper.setHopper(state);
                            }
                            elevator.runElevator();
                        }
                    }
                }

                // TODO: choose one place to call runHopper
                //  either in endeffector periodic or in here but right now it runs in both
                hopper.runHopper();
            },
            this
        );
    }

    public Command toggleManualControl() {
        return runOnce(() -> {
            this.isManualControlEnabled = !this.isManualControlEnabled;
            this.elevator.setManualControlEnabled(isManualControlEnabled);
            this.hopper.setManualControlEnabled(isManualControlEnabled);
        });
    }

    /**
     * @return whether both "sub-subsystems" at the specified position
     */
    public boolean isAtPosition() {
        return elevator.isAtTarget() && hopper.isAtTarget();
    }

    public Trigger isAtPosition = new Trigger(this::isAtPosition);

    /**
     * @return whether the state is finished. Note this is different from {@link ScoringSuperstructure#isAtPosition()},
     * since the state could only be finished when a game piece is detected, but we might only start the intake wheels when
     * {@link ScoringSuperstructure#isAtPosition()} returns true.
     */
    public boolean isStateFinished() {
        return elevator.isElevatorStateFinished() && hopper.isHopperStateFinished();
    }

    public Trigger isStateFinished = new Trigger(this::isStateFinished);

    /**
     * automatically detects whether current state was "aborted" or is finished, then sets the current state to the next
     * state if so.
     */
    @Override
    public void periodic() {
        if (isStateFinished()) {
            setState(state.getStateAfter());
        } else if (RobotState.isTeleop() && !state.control.getAsBoolean()) {
            setState(ScoringSuperstructureState.IDLE);
        }

        //Sets scoring mechanisms to IDLE in case robot acceleration is high.
        if (SubsystemManager.getInstance().getDrivetrain().getAccelerationInGs() >= .4) {
            setState(ScoringSuperstructureState.IDLE);
        }
        SmartDashboard.putBoolean("isManualControlEnabled", isManualControlEnabled);
    }

    /**
     * @return the real life length of the elevator, for use in simulation only.
     */
    public Distance getCurrentElevatorLength() {
        return elevator.getCurrentHeight();
    }

    /**
     * @return the real life target length of the elevator, for use in simulation only.
     */
    public Distance getTargetElevatorLength() {
        return elevator.getTargetHeight();
    }

    /**
     * @return the real life rotation of the wrist, for use in simulation only.
     */
    public Rotation2d getCurrentWristRotation() {
        return hopper.getCurrentRotation();
    }

    /**
     * @return the real life target rotation of the wrist, for use in simulation only.
     */
    public Rotation2d getTargetWristRotation() {
        return hopper.getTargetRotation();
    }

    public Command elevatorSysIDQuasistatic(SysIdRoutine.Direction direction) {
        return ElevatorSysID.sysIdQuasistatic(direction);
    }

    public Command elevatorSysIDDynamic(SysIdRoutine.Direction direction) {
        return ElevatorSysID.sysIdDynamic(direction);
    }
}