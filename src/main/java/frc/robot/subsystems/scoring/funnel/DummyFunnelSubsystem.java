package frc.robot.subsystems.scoring.funnel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

public class DummyFunnelSubsystem extends AbstractFunnelSubsystem{
    @Override
    public boolean isFunnelStateFinished() {
        return true;
    }

    @Override
    public void setFunnel(ScoringSuperstructureState state) {
        this.state = state;
    }

    @Override
    protected void runFunnelPosition() {}
}
