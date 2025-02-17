package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

public class PoseUtil {
    public static Translation2d translationInDirection(Translation2d translation, Rotation2d angle) {
        return translation.rotateBy(angle);
    }

    public static Pose2d ReefRelativeLeftOf(Pose2d pose, Rotation2d angle) {
        return new Pose2d(
                pose.getTranslation().plus(new Translation2d(FieldConstants.reefForwardsDistance, FieldConstants.reefSidewaysDistance)),
                pose.getRotation()
        );
    }

    public static Pose2d ReefRelativeRightOf(Pose2d pose, Rotation2d angle) {
        return new Pose2d(
                pose.getTranslation().plus(new Translation2d(FieldConstants.reefForwardsDistance, -FieldConstants.reefSidewaysDistance)),
                pose.getRotation()
        );
    }

    public static Rotation2d getReefAngle(FieldConstants.TargetPositions targetPosition) {
        return switch (targetPosition) {
            case REEF_AB -> Rotation2d.fromDegrees(0);
            case REEF_KL -> Rotation2d.fromDegrees(-60);
            case REEF_IJ -> Rotation2d.fromDegrees(-120);
            case REEF_GH -> Rotation2d.fromDegrees(-180);
            case REEF_EF -> Rotation2d.fromDegrees(-240);
            case REEF_CD -> Rotation2d.fromDegrees(-300);
            default -> new Rotation2d();
        };
    }

    public static Pose2d ReefRelativeLeftOf(FieldConstants.TargetPositions targetPosition) {
        return ReefRelativeLeftOf(
                targetPosition.Pose,
                getReefAngle(targetPosition)
        );
    }

    public static Pose2d ReefRelativeRightOf(FieldConstants.TargetPositions targetPosition) {
        return ReefRelativeRightOf(
                targetPosition.Pose,
                getReefAngle(targetPosition)
        );
    }
}
