package frc.robot.subsystems;

import frc.robot.subsystems.climber.AbstractClimberSubsystem;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Objects;

/**
 * A single access point for all subsystems
 */
public class SubsystemManager {
    private static volatile SubsystemManager instance;

    private AbstractClimberSubsystem climberSubsystem;
    private AbstractSwerveDrivetrain drivetrain;
    private ScoringSuperstructure scoringSuperstructure;
    private VisionSubsystem visionSubsystem;

    public static synchronized SubsystemManager getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, SubsystemManager::new);
    }

    private SubsystemManager() {}

    public void instantiateSubsystems() {
        climberSubsystem = AbstractClimberSubsystem.getInstance();
        drivetrain = AbstractSwerveDrivetrain.getInstance();
        scoringSuperstructure = ScoringSuperstructure.getInstance();
        visionSubsystem = VisionSubsystem.getInstance();
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

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
}
