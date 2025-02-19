package frc.robot.subsystems.scoring.endeffector;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.scoring.ScoringSuperstructureAction;

public class DummyEndEffectorSubsystem extends AbstractEndEffectorSubsystem {
    @Override
    public Rotation2d getCurrentRotation() {
        return state.getRotation();
    }

    @Override
    public double getIntakeSpeed() {
        return 0;
    }

    @Override
    public boolean isWristAtTarget() {
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
    public void setHopper(ScoringSuperstructureAction state) {
        this.state = state;
    }

    @Override
    protected void runHopperPosition() {}

    @Override
    public void runHopper() {}
}
