package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;

public class AllianceFlipUtil {

    public static double applyX(double x) {
        return FieldConstants.fieldLength - x;
    }

    public static double applyY(double y) {
        return FieldConstants.fieldWidth - y;
    }

    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return rotation.rotateBy(Rotation2d.kPi);
    }

    public static Pose2d apply(Pose2d pose) {
        return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    public static boolean shouldFlip() {
        return DriverStationUtil.getAlliance() != DriverStation.Alliance.Red;
    }

    public static Pose2d applyIfShouldFlip(Pose2d pose){
        return shouldFlip() ? apply(pose) : pose;
    }

    /**
     * Always alliance-flips a pose without checking Driver Station. Useful when working with
     * a pose that is already alliance-normalized
     * and needs to be flipped in all situations.
     * @param pose
     * @return
     */
    public static Pose2d rawAllianceFlipPose(Pose2d pose) {
        return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }
}