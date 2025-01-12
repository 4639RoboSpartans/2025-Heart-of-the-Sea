package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public abstract class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            if (Robot.isReal())
                instance = new ConcreteElevatorSubsystem();
            else
                instance = new SimElevatorSubsystem();
        }
        return instance;
    }

    public abstract void setElevator(ArmSuperstructureState state, GamePiece gamePiece);

    public abstract void runElevator();

    protected abstract boolean atState();

    public abstract double getCurrentPosition();

    public Trigger atRequestedStateTrigger() {
        return new Trigger(this::atState);
    }

    public abstract double getTargetPosition();

    public static double getElevatorPosition(ArmSuperstructureState state, GamePiece gamePiece) {
        return switch (state) {
            case IDLE -> ArmConstants.ElevatorConstants.DOWN_POSITION;
            case GROUND_INTAKING, OUTTAKING -> ArmConstants.ElevatorConstants.GROUND_INTAKING_POSITION;
            case SUBSTATION_INTAKING -> ArmConstants.ElevatorConstants.SUBSTATION_INTAKING_POSITION;
            case LOW -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.ElevatorConstants.CONE_POSITIONS[0] :
                    ArmConstants.ElevatorConstants.CUBE_POSITIONS[0];
            case MID -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.ElevatorConstants.CONE_POSITIONS[1] :
                    ArmConstants.ElevatorConstants.CUBE_POSITIONS[1];
            case HIGH -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.ElevatorConstants.CONE_POSITIONS[2] :
                    ArmConstants.ElevatorConstants.CUBE_POSITIONS[2];
        };
    }

    public abstract Command quasistatic(SysIdRoutine.Direction direction);

    public abstract Command dynamic(SysIdRoutine.Direction direction);
}
