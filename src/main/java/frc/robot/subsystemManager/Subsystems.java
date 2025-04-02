package frc.robot.subsystemManager;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.climber.AbstractClimberSubsystem;
import frc.robot.subsystems.climber.ServoSubsystem;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.drive.LasercanAlign;
import frc.robot.subsystems.led.LEDStrip;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.elevator.AbstractElevatorSubsystem;
import frc.robot.subsystems.scoring.endeffector.AbstractEndEffectorSubsystem;

/**
 * A single access point for all subsystems in the robot.
 */
public class Subsystems {
    private static final AbstractClimberSubsystem climberSubsystem;
    private static final AbstractSwerveDrivetrain drivetrain;
    private static final AbstractElevatorSubsystem elevator;
    private static final AbstractEndEffectorSubsystem endEffector;
    private static final ScoringSuperstructure scoringSuperstructure;
    private static final LEDStrip ledStripSubsystem;
    private static final LasercanAlign lasercanAlignSubsystem;
    private static final ServoSubsystem servoSubsystem;

    private Subsystems() {}

    static {
        climberSubsystem = AbstractClimberSubsystem.getInstantiator().instantiateSubsystem();
        drivetrain = AbstractSwerveDrivetrain.getInstantiator().instantiateSubsystem();
        // Elevator and EndEffector must be instantiated before scoringSuperstructure
        elevator = AbstractElevatorSubsystem.getInstantiator().instantiateSubsystem();
        endEffector = AbstractEndEffectorSubsystem.getInstantiator().instantiateSubsystem();
        scoringSuperstructure = ScoringSuperstructure.getInstantiator().instantiateSubsystem();
        ledStripSubsystem = LEDStrip.createInstance().instantiateSubsystem();
        lasercanAlignSubsystem = LasercanAlign.createInstance().instantiateSubsystem();
        servoSubsystem = ServoSubsystem.createInstance().instantiateSubsystem();
    }

    private static <S extends Subsystem> S maybeGet(S subsystem, Class<S> cls) {
        if (subsystem == null) throw new RuntimeException(
            "Subsystem %s was requested but is not yet instantiated, check instantiation order in SubsystemManager"
                .formatted(cls.getSimpleName())
        );
        return subsystem;
    }


    public static AbstractClimberSubsystem climber() {
        return maybeGet(climberSubsystem, AbstractClimberSubsystem.class);
    }

    public static AbstractSwerveDrivetrain drivetrain() {
        return maybeGet(drivetrain, AbstractSwerveDrivetrain.class);
    }

    public static AbstractElevatorSubsystem elevator() {
        return maybeGet(elevator, AbstractElevatorSubsystem.class);
    }

    public static AbstractEndEffectorSubsystem endEffector() {
        return maybeGet(endEffector, AbstractEndEffectorSubsystem.class);
    }

    public static ScoringSuperstructure scoringSuperstructure() {
        return maybeGet(scoringSuperstructure, ScoringSuperstructure.class);
    }

    public static LEDStrip ledStrip() {
        return maybeGet(ledStripSubsystem, LEDStrip.class);
    }

    public static LasercanAlign lasercanAlign() {
        return maybeGet(lasercanAlignSubsystem, LasercanAlign.class);
    }

    public static ServoSubsystem servo() {
        return maybeGet(servoSubsystem, ServoSubsystem.class);
    }
}
