package frc.robot.subsystems;

import frc.lib.led.LEDStrip;
import frc.lib.led.PhysicalLEDStrip;
import frc.robot.subsystems.climber.AbstractClimberSubsystem;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;

import java.util.Objects;

/**
 * A single access point for all subsystems
 */
public class SubsystemManager {
    private static volatile SubsystemManager instance;

    public static final class GetInstanceAccess {
        private GetInstanceAccess() {}
    }
    private static final GetInstanceAccess getInstanceAccess = new GetInstanceAccess();

    private AbstractClimberSubsystem climberSubsystem;
    private AbstractSwerveDrivetrain drivetrain;
    private ScoringSuperstructure scoringSuperstructure;
    private LEDStrip ledStripSubsystem;

    public static synchronized SubsystemManager getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, SubsystemManager::new);
    }

    static {
        instance = getInstance();
        instance.instantiateSubsystems();
    }

    private SubsystemManager() {}

    public void instantiateSubsystems() {
        //TODO: the climber doesnt exist so i will probably dummy it out
        //climberSubsystem = AbstractClimberSubsystem.getInstance();
        drivetrain = AbstractSwerveDrivetrain.getInstance(getInstanceAccess);
        scoringSuperstructure = ScoringSuperstructure.getInstance(getInstanceAccess);
        ledStripSubsystem = LEDStrip.getInstance();
    }

    public AbstractClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public AbstractSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public ScoringSuperstructure getScoringSuperstructure() {
        return scoringSuperstructure;
    }

    public LEDStrip getLEDStripSubsystem() {
        return ledStripSubsystem;
    }

}
