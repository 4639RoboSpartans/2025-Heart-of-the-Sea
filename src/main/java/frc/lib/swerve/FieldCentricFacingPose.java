package frc.lib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldCentricFacingPose extends SwerveRequest.FieldCentricFacingAngle {
    private Pose2d targetPose = new Pose2d();
    private Pose2d robotPose = new Pose2d();

    public FieldCentricFacingPose withTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        return this;
    }

    public FieldCentricFacingPose withRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
        return this;
    }

    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Translation2d targetTranslation = targetPose.getTranslation();
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d translationDiff = targetTranslation.minus(robotTranslation);
        Rotation2d angleToFace = new Rotation2d(translationDiff.getY(), translationDiff.getX());
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            /* If we're facing the operator perspective, rotate the direction we want to face by the angle */
            angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
        }

        double toApplyOmega = TargetRateFeedforward +
                HeadingController.calculate(
                        parameters.currentPose.getRotation().getRadians(),
                        angleToFace.getRadians(),
                        parameters.timestamp
                );

        return new SwerveRequest.FieldCentric()
                .withVelocityX(VelocityX)
                .withVelocityY(VelocityY)
                .withRotationalRate(toApplyOmega)
                .withDeadband(Deadband)
                .withRotationalDeadband(RotationalDeadband)
                .withCenterOfRotation(CenterOfRotation)
                .withDriveRequestType(DriveRequestType)
                .withSteerRequestType(SteerRequestType)
                .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
                .withForwardPerspective(ForwardPerspective)
                .apply(parameters, modulesToApply);
    }
}
