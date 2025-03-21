package frc.robot.subsystems.scoring.endeffector;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SubsystemManager;

public class DummyEndEffectorSubsystem extends AbstractEndEffectorSubsystem {

    @Override
    public Rotation2d getCurrentRotation() {
        return getTargetRotation();
    }

    @Override
    public boolean hasCoral() {
        return SubsystemManager.getInstance().getScoringSuperstructure().getCurrentAction().endOnGamePieceSeen;
    }

    @Override
    protected void periodic(double targetWristRotationFraction, double intakeSpeed) {
    }

    @Override
    public void setWristMotorIdleMode(SparkBaseConfig.IdleMode mode) {

    }

    @Override
    public boolean isWristPhysicallyStopped() {
        return true;
    }

    @Override
    public void resetCurrentWristRotationFractionTo(double newWristRotationFraction) {}
}
