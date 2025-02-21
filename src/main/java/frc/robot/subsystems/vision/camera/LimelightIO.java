package frc.robot.subsystems.vision.camera;

import java.util.Arrays;
import java.util.Collection;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.DriverStationUtil;
import frc.lib.network.LimelightHelpers;
import frc.lib.network.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.vision.VisionResult;

/**
 * Wraps a Limelight in the shape of a CameraIO interface, to be used in the Vision subsystem.
 */
public class LimelightIO implements CameraIO {
    private final String name;
    PoseEstimate lastPoseEstimate;

    /**
     * Constructs a new LimelightIO object.
     * @param name the nickname of the camera. This will be the same as the NetworkTable the camera broadcasts to.
     */
    public LimelightIO(String name){
        this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Optional<VisionResult> getBotPoseAsVisionResult(boolean allianceFlipped) {
        LimelightHelpers.SetRobotOrientation(name, SubsystemManager.getInstance().getDrivetrain().getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate poseEstimate = DriverStationUtil.getAlliance() == Alliance.Blue || !allianceFlipped
                                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
                                        : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
        if(SubsystemManager.getInstance().getDrivetrain().getPose().equals(new Pose2d()) || verifyPose(poseEstimate.pose, allianceFlipped).isPresent()){
            lastPoseEstimate = poseEstimate;
            return Optional.of(new VisionResult(poseEstimate.pose, poseEstimate.timestampSeconds));    
        } 
        return Optional.empty();
    }

    private Optional<Pose2d> verifyPose(Pose2d measurement, boolean allianceFlipped){
        return (measurement.getX() == 0 || measurement.getY() == 0
            ? Optional.empty()
            : (measurement.getTranslation().getDistance(SubsystemManager.getInstance().getDrivetrain().getPose().getTranslation()) <= 1
                ? Optional.of(measurement)
                : Optional.empty())
            );
    }

    @Override
    public Collection<Integer> targets() {
        return Arrays.stream(lastPoseEstimate.rawFiducials).parallel().map(rawFiducial -> rawFiducial.id).collect(Collectors.toSet());
    }
    
}
