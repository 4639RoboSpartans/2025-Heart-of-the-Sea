package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Wraps a Pose2d estimate and a timestamp in an object.
 * <p>
 * Used to pass information from cameras to the vision system.
 */
public record VisionResult(Pose2d estimatedRobotPose, double timestamp) {}
