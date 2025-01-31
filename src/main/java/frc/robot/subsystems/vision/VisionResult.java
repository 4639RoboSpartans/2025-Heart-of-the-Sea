package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Wraps a Pose2d estimate and a timestamp in an object.
 * <p>
 * Used to pass information from cameras to the vision system.
 */
public class VisionResult {
    private Pose2d visionPose;
    private double timestamp;

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
