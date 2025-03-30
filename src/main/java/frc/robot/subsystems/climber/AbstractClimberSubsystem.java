package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.SubsystemManager;

import java.util.function.DoubleSupplier;

public abstract class AbstractClimberSubsystem extends SubsystemBase {
    abstract void setClimberSpeed(double speed);
    abstract ClimberState getClimberState();
    abstract void setClimberState(ClimberState state);
    abstract double getEncoderPosition();

    enum ClimberState {
        STOWED,
        CLIMBER_READY,
        FUNNEL_READY,
        READY,
        CLIMBING
    }

    void init(){
        //reset state to idle when auto or teleop starts
        RobotModeTriggers.autonomous().onTrue(setState(ClimberState.STOWED));
        RobotModeTriggers.teleop().onTrue(setState(ClimberState.STOWED));

        //put override on Smart Dashboard
        SmartDashboard.putBoolean("climb/Climber Override", false);
    }

    public static AbstractClimberSubsystem getInstance() {
        return RobotBase.isReal() ? ConcreteClimberSubsystem.getInstance() : SimClimberSubsystem.getInstance();
    }

    public Command stopClimber() {
        return Commands.run(
                () -> setClimberSpeed(0),
                this
        );
    }

    public Command climbCommand() {
        return setState(ClimberState.CLIMBING)
                .andThen(run(
                        () ->
                        {
                            // if (ClimberConstants.Setpoints.climbPosition.get() < getEncoderPosition()) {
                            //     setClimberSpeed(0);
                            // } else {
                            setClimberSpeed(ClimberConstants.climberSpeed.get());
                            // }
                        }
                ));
    }

    public Command deClimbCommand() {
        return setState(ClimberState.CLIMBING)
                .andThen(run(
                        () ->
                        {
                            setClimberSpeed(-ClimberConstants.climberSpeed.get());
                        }
                ));
    }

    public Command prepClimbCommand() {
        return setState(getClimberState() == ClimberState.FUNNEL_READY ? ClimberState.READY : ClimberState.CLIMBER_READY)
                .andThen(run(
                        () ->
                        {
                            if (ClimberConstants.Setpoints.readyToClimbPosition.get() > getEncoderPosition()) {
                                setClimberSpeed(0);
                            } else {
                                setClimberSpeed(ClimberConstants.climberSpeed.get());
                            }
                        }
                ));
    }

    public Command testClimbCommand(DoubleSupplier speed) {
        return Commands.run(
                () -> setClimberSpeed(speed.getAsDouble() * ClimberConstants.climberSpeed.get()),
                this
        );
    }

    Command setState(ClimberState state) {
        return runOnce(() -> setClimberState(state));
    }

    public static boolean funnelDropAllowed() {
        return getInstance().getClimberState().equals(ClimberState.STOWED) && (DriverStation.getMatchTime() <= 30 || SmartDashboard.getBoolean("climber/Climber Override", false));
    }

    public static boolean readyToClimb() {
        return getInstance().getClimberState().equals(ClimberState.READY) || getInstance().getClimberState().equals(ClimberState.CLIMBING);
    }

    @Override
    public void periodic() {
        if (getClimberState() == ClimberState.STOWED) {
            if (SubsystemManager.getInstance().getServoSubsystem().getServoPosition() == -1.0) {
                setClimberState(ClimberState.FUNNEL_READY);
            }
        } else if (getClimberState() == ClimberState.CLIMBER_READY) {
            if (SubsystemManager.getInstance().getServoSubsystem().getServoPosition() == -1.0) {
                setClimberState(ClimberState.READY);
            }
        }
        SmartDashboard.putNumber("climb/encoder", getEncoderPosition());
    }
}