package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

public class PoseUtil {
    public static Translation2d translationInDirection(Translation2d translation, Rotation2d angle) {
        return translation.rotateBy(angle);
    }
    
    public static Pose2d leftOf(Pose2d pose, Rotation2d angle) {
        return new Pose2d(
                pose.getTranslation()
                        .plus(translationInDirection(
                                new Translation2d(
                                        FieldConstants.reefSidewaysDistance,
                                        FieldConstants.reefForwardsDistance
                                ), 
                                angle
                        )), 
                pose.getRotation()
        );
    }

    public static Pose2d rightOf(Pose2d pose, Rotation2d angle) {
        return new Pose2d(
                pose.getTranslation()
                        .plus(translationInDirection(
                                new Translation2d(
                                        FieldConstants.reefSidewaysDistance,
                                        -FieldConstants.reefForwardsDistance
                                ),
                                angle
                        )),
                pose.getRotation()
        );
    }

    public static Rotation2d getReefAngle(FieldConstants.TargetPositions targetPosition) {
        return switch (targetPosition) {
            case REEF_0 -> Rotation2d.fromDegrees(0);
            case REEF_1 -> Rotation2d.fromDegrees(-60);
            case REEF_2 -> Rotation2d.fromDegrees(-120);
            case REEF_3 -> Rotation2d.fromDegrees(-180);
            case REEF_4 -> Rotation2d.fromDegrees(-240);
            case REEF_5 -> Rotation2d.fromDegrees(-300);
            default -> new Rotation2d();
        };
    }

    public static Pose2d leftOf(FieldConstants.TargetPositions targetPosition) {
        return leftOf(
                targetPosition.Pose,
                getReefAngle(targetPosition)
        );
    }

    public static Pose2d rightOf(FieldConstants.TargetPositions targetPosition) {
        return rightOf(
                targetPosition.Pose,
                getReefAngle(targetPosition)
        );
    }
}
