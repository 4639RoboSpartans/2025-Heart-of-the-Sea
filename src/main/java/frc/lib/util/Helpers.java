package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Helper class that contains static methods for common operations that don't fit anywhere else.
 */

@SuppressWarnings("unused")
public class Helpers {

    /**
     * Returns true if the translation is within the specified tolerance.
     * @param translation The Translation2d to check.
     * @param maxErrorMeters The maximum error in meters.
     * @return If the translation is within the specified tolerance.
     */
    private static boolean withinTolerance(Translation2d translation, double maxErrorMeters) {
        return Math.abs(translation.getX()) <= maxErrorMeters && Math.abs(translation.getY()) <= maxErrorMeters;
    }

    private static boolean withinTolerance(double measurement, double expected, double maxError) {
        return Math.abs(measurement - expected) <= maxError;
    }

    /**
     * Returns the angle (in degrees) between the given robot position and the goal.
     * @param robotPose The robot's current position.
     * @param goal The goal position.
     * @return The angle (in degrees) between the robot and the goal.
     */
    public static double angleToGoal(Pose2d robotPose, Translation2d goal) {
        Translation2d robotToGoal = goal.minus(robotPose.getTranslation());
        double angle = Math.atan2(robotToGoal.getY(), robotToGoal.getX());
        return Math.toDegrees(angle);
    }
}
