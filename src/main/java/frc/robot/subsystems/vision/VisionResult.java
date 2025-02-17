package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

// TODO: Look into Java record classes. Would
//  :
//  public record VisionResult(Pose2d visionPose, double timestamp) {}
//  :
//  be sufficient? Also, you can still add methods in record classes, in case that is needed later.

/**
 * Wraps a Pose2d estimate and a timestamp in an object.
 * <p>
 * Used to pass information from cameras to the vision system.
 */
public class VisionResult {
    private final Pose2d visionPose;
    private final double timestamp;

    public VisionResult(Pose2d visionPose, double timestamp){
        this.visionPose = visionPose;
        this.timestamp = timestamp;
    }

    public Pose2d getVisionPose(){
        return visionPose;
    }

    public double getTimestamp(){
        return timestamp;
    }
}
