package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DummySwerveDrivetrain extends AbstractSwerveDrivetrain {
    Pose2d currentPose = new Pose2d();

    @Override
    public Command resetPigeon() {
        return new InstantCommand(() -> {});
    }

    @Override
    public Command stop() {
        return new InstantCommand(() -> {});
    }

    @Override
    public void setControl(SwerveRequest control) {}

    @Override
    public Command manualControl() {
        return new InstantCommand(() -> {});
    }

    @Override
    public Command directlyMoveTo(Pose2d targetPose) {
        return new InstantCommand(() -> {
            currentPose = targetPose;
        });
    }

    @Override
    public Command pathfindTo(Pose2d targetPose) {
        return new InstantCommand(() -> {
            currentPose = targetPose;
        });
    }

    @Override
    public void followPath(SwerveSample sample) {
        currentPose = sample.getPose();
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    @Override
    public double getAccelerationInGs() {
        return 0;
    }

    @Override
    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp) {

    }

    @Override
    public boolean atTargetPose(Pose2d targetPose) {
        return true;
    }

    @Override
    public void setVisionStandardDeviations(double xStdDev, double yStdDev, double rotStdDev) {
       
    }
}
