package frc.robot.subsystems.climber;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ForSubsystemManagerUseOnly;
import frc.robot.subsystems.SubsystemManager;

public abstract class AbstractClimberSubsystem extends SubsystemBase {
    private static AbstractClimberSubsystem instance;

    /**
     * This method should only be accessed from the SubsystemManager class. In other places, use
     * {@link SubsystemManager#getClimberSubsystem()} instead.
     */
    @ForSubsystemManagerUseOnly
    public static AbstractClimberSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcreteClimberSubsystem::new);
    }

    public abstract Command runClimber(DoubleSupplier speedSupplier);
}
