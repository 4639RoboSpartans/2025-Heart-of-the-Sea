package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSuperstructure;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import java.util.Objects;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;

    // TODO: Don't do this! Singleton getInstance() should never take parameters.
    //  Side note: if you really do need different instances for different parameters, use some sort of map from
    //  parameter to instance, and name it getInstanceFor(param1, param2, ...)
    public static HopperSubsystem getInstance(ScoringSuperstructure scoringSuperstructure) {
        if (Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(
                    instance,
                    () -> new ConcreteHopperSubsystem(scoringSuperstructure)
            );
        } else {
            return instance = Objects.requireNonNullElseGet(
                    instance,
                    () -> new SimHopperSubsystem(scoringSuperstructure)
            );
        }
    }

    public abstract double getCurrentPosition();

    public abstract Rotation2d getCurrentRotation();

    public abstract double getTargetPosition();

    public abstract Rotation2d getTargetRotation();

    public abstract boolean isHopperAtPosition();
    public Trigger isHopperAtPosition = new Trigger(this::isHopperAtPosition);

    public abstract boolean isHopperStateFinished();
    public Trigger isHopperStateFinished = new Trigger(this::isHopperStateFinished);

    public abstract boolean hasCoral();
    public Trigger hasCoral = new Trigger(this::hasCoral);

    public abstract void setHopper(ScoringSuperstructureState state);

    protected abstract void runHopperPosition();

    public abstract void runHopper();
}
