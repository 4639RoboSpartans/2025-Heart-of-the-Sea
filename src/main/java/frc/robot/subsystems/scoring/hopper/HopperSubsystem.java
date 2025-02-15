package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot.Robot;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;

import java.util.Objects;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;

    //Turns out, there is never more than one ScoringSuperstructure (good)
    //which means there is no reason to pass in the ScoringSuperstructure object
    //which means we do this instead
    public static HopperSubsystem getInstance() {
        //doing this to stop the hopper from existing before its ready without commenting out a bunch of code
        // (also, please multiline the comments instead of commenting out every line individually)
        //TODO: remove the false flag when hopper is ready
        if (false && Robot.isReal()) {
            return instance = Objects.requireNonNullElseGet(
                instance,
                ConcreteHopperSubsystem::new
            );
        } else {
            return instance = Objects.requireNonNullElseGet(
                instance,
                SimHopperSubsystem::new
            );
        }
    }

    protected ScoringSuperstructureState state = ScoringSuperstructureState.IDLE;

    public final ScoringSuperstructureState getHopperState() {
        return state;
    }

    public abstract Rotation2d getCurrentRotation();

    public final double getCurrentPosition() {
        return ScoringConstants.HopperConstants.PositionToRotation.convertBackwards(getCurrentRotation());
    }

    public final Rotation2d getTargetRotation() {
        return state.getRotation();
    }

    public final double getTargetPosition() {
        return ScoringConstants.HopperConstants.PositionToRotation.convertBackwards(getTargetRotation());
    }

    public abstract double getIntakeSpeed();

    public abstract boolean isAtTarget();

    public abstract boolean isHopperStateFinished();

    public abstract boolean hasCoral();

    public Trigger hasCoral = new Trigger(this::hasCoral);

    public abstract void setHopper(ScoringSuperstructureState state);

    protected abstract void runHopperPosition();

    public abstract void runHopper();
}
