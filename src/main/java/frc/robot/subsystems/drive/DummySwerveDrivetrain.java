package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

public class DummySwerveDrivetrain extends AbstractSwerveDrivetrain {
    Pose2d currentPose = new Pose2d();

    @Override
    public Command resetHeadingToZero() {
        return Commands.none();
    }

    @Override
    public Command stop() {
        return Commands.none();
    }

    @Override
    public void setControl(SwerveRequest control) {}

    @Override
    public Command manualControl() {
        return Commands.none();
    }

    @Override
    public Command _directlyMoveTo(Pose2d targetPose, Supplier<Pose2d> currentPoseSupplier) {
        return Commands.runOnce(() -> {
            currentPose = targetPose;
        });
    }

    @Override
    public Command fineTuneUsingLaserCANCommand(Pose2d targetPose) {
        return Commands.runOnce(() -> {
            currentPose = targetPose;
        });
    }

    @Override
    public Command pathToPoseCommand(Pose2d targetPose) {
        return Commands.none();
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
    public boolean nearTargetPose(Pose2d targetPose) {
        return true;
    }

    @Override
    public void setVisionStandardDeviations(double xStdDev, double yStdDev, double rotStdDev) {
       
    }

    @Override
    public double[] getVisionStandardDeviations() {
        return new double[]{0, 0, 0};
    }

    @Override
    public OptionalDouble getDistanceFromReefFace() {
        return OptionalDouble.empty();
    }

    @Override
    public Optional<Rotation2d> getCalculatedRotationFromAlign() {
        return Optional.empty();
    }
    
    @Override
    public boolean isAligned() {
        return false;
    }

    @Override
    public Command toggleAutoHeading() {
        return Commands.none();
    }
}
