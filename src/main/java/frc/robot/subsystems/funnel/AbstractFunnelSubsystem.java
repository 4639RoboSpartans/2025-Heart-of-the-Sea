package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystems.SubsystemManager;

import java.util.Objects;

public abstract class AbstractFunnelSubsystem extends SubsystemBase {
    private static AbstractFunnelSubsystem instance;

    public static AbstractFunnelSubsystem getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);
        return instance = Objects.requireNonNullElseGet(instance, Robot.isReal() ?
            ConcreteFunnelSubsystem::new :
            DummyFunnelSubsystem::new // TODO: add sim funnel
        );
    }

    protected boolean active = false;

    /**
     * Sets whether the funnel is active
     *
     * @param active Whether the funnel is in intaking position
     */
    public final void setFunnelActive(boolean active) {
        this.active = active;
    }
}
