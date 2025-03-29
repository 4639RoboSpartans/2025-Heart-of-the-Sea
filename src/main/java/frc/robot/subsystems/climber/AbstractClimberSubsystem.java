package frc.robot.subsystems.climber;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public abstract class AbstractClimberSubsystem extends SubsystemBase {
    abstract void setClimberSpeed(double speed);
    abstract ClimberState getClimberState();
    abstract void setClimberState(ClimberState state);
    abstract void setServoPosition(double servoPosition);
    abstract double getServoPosition();
    abstract double getEncoderPosition();

    private final PIDController climberPID = new PIDController(ClimberConstants.PIDs.climbKp.get(), 0, 0);

    static enum ClimberState {
        STOWED,
        CLIMBER_READY,
        FUNNEL_READY,
        READY,
        CLIMBING;
    }

    void init(){
        //reset state to idle when auto or teleop starts
        RobotModeTriggers.autonomous().onTrue(setState(ClimberState.STOWED));
        RobotModeTriggers.teleop().onTrue(setState(ClimberState.STOWED));

        //put override on Smart Dashboard
        SmartDashboard.putBoolean("climber/Climber Override", false);
        climberPID.enableContinuousInput(0, 1);
    }

    public static AbstractClimberSubsystem getInstance() {
        return RobotBase.isReal() ? ConcreteClimberSubsystem.getInstance() : SimClimberSubsystem.getInstance();
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }

    public Command climbCommand() {
        return setState(ClimberState.CLIMBING)
        .andThen(run(
            () -> 
                {
                    climberPID.setSetpoint(ClimberConstants.Setpoints.climbPosition.get());
                    setClimberSpeed(climberPID.calculate(getEncoderPosition()));
                }
        ));
    }

    public Command prepClimbCommand() {
        return setState(getClimberState() == ClimberState.FUNNEL_READY ? ClimberState.READY : ClimberState.CLIMBER_READY)
        .andThen(run(
            () -> 
                {
                    climberPID.setSetpoint(ClimberConstants.Setpoints.readyToClimbPosition.get());
                    setClimberSpeed(climberPID.calculate(getEncoderPosition()));
                }
        )).until(() -> climberPID.atSetpoint())
        .andThen(Commands.idle(this))
        .finallyDo(this::stopClimber);
    }

    public Command idleClimbCommand() {
        return run(
            () -> 
                {
                    climberPID.setSetpoint(getEncoderPosition());
                    setClimberSpeed(climberPID.calculate(getEncoderPosition()));
                }
        );
    }

    public Command dropFunnel(){
        return runOnce(() -> setServoPosition(ClimberConstants.Setpoints.dropPosition.get()))
                .andThen(setState(getClimberState() == ClimberState.CLIMBER_READY ? ClimberState.READY : ClimberState.FUNNEL_READY));
    }

    public Command bindFunnel() {
        return runOnce(() -> setServoPosition(ClimberConstants.Setpoints.holdingPosition.get()))
                .andThen(setState(ClimberState.STOWED));
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

    public static double reMap(double zero, double input) {
        if (input < zero) return 1 + input - zero;
        else return input - zero;
    }
}
