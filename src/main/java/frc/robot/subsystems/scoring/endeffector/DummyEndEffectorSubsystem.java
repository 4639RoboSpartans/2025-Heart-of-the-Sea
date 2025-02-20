package frc.robot.subsystems.scoring.endeffector;

import edu.wpi.first.math.geometry.Rotation2d;

public class DummyEndEffectorSubsystem extends AbstractEndEffectorSubsystem {

    @Override
    public Rotation2d getCurrentRotation() {
        return getTargetRotation();
    }

    @Override
    public boolean hasCoral() {
        return false;
    }

    @Override
    protected void periodic(double targetWristRotationFraction, double intakeSpeed) {}
}
