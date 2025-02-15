package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public class DummyHopperSubsystem extends HopperSubsystem {
    @Override
    public Rotation2d getCurrentRotation() {
        return state.getRotation();
    }

    @Override
    public double getIntakeSpeed() {
        return 0;
    }

    @Override
    public boolean isAtTarget() {
        return true;
    }

    @Override
    public boolean isHopperStateFinished() {
        return true;
    }

    @Override
    public boolean hasCoral() {
        return false;
    }

    @Override
    public void setHopper(ScoringSuperstructureState state) {
        this.state = state;
    }

    @Override
    protected void runHopperPosition() {}

    @Override
    public void runHopper() {}
}
