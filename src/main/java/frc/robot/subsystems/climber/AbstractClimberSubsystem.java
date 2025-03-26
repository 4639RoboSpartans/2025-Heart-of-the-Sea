package frc.robot.subsystems.climber;
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

    static enum ClimberState{
        IDLE,
        READY,
        CLIMBING;
    }

    void init(){
        //reset state to idle when auto or teleop starts
        RobotModeTriggers.autonomous().onTrue(setState(ClimberState.IDLE));
        RobotModeTriggers.teleop().onTrue(setState(ClimberState.IDLE));

        //put override on Smart Dashboard
        SmartDashboard.putBoolean("climber/Climber Override", false);
    }

    public static AbstractClimberSubsystem getInstance(){
        return RobotBase.isReal() ? ConcreteClimberSubsystem.getInstance() : SimClimberSubsystem.getInstance();
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }

    public Command climberUp(){
        return runOnce(() -> setClimberSpeed(ClimberConstants.climberSpeed.get()))
                .andThen(setState(ClimberState.CLIMBING))
                .andThen(Commands.idle(this))
                .finallyDo(this::stopClimber);
    }

    public Command climberDown(){
        return runOnce(() -> setClimberSpeed(-ClimberConstants.climberSpeed.get()))
                .andThen(setState(ClimberState.READY))
                .andThen(Commands.idle(this))
                .finallyDo(this::stopClimber);
    }

    public Command dropFunnel(){
        return runOnce(() -> setServoPosition(ClimberConstants.ServoSetpoints.dropPosition.get()))
                .andThen(setState(ClimberState.READY));
    }

    public Command bindFunnel(){
        return runOnce(() -> setServoPosition(ClimberConstants.ServoSetpoints.holdingPosition.get()))
                .andThen(setState(ClimberState.IDLE));
    }

    Command setState(ClimberState state){
        return runOnce(() -> setClimberState(state));
    }

    public static boolean funnelDropAllowed(){
        return getInstance().getClimberState().equals(ClimberState.IDLE) && (DriverStation.getMatchTime() <= 30 || SmartDashboard.getBoolean("climber/Climber Override", false));
    }

    public static boolean readyToClimb() {
        return getInstance().getClimberState().equals(ClimberState.READY) || getInstance().getClimberState().equals(ClimberState.CLIMBING);
    }
}
