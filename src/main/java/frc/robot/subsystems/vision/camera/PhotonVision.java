package frc.robot.subsystems.vision.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.DriverStationHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Wraps a PhotonCamera in the shape of a CameraIO interface, to be used in the Vision subsystem.
 */
public class PhotonVision implements Camera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    EstimatedRobotPose lastPoseEstimate;

    /**
     * Constructs a new PhotonVisionIO object.
     *
     * @param name                     the nickname of the camera. This will be the same as the NetworkTable the camera broadcasts to.
     * @param transformFromRobotOrigin the offsets from the camera's position to the robot's position.
     */
    public PhotonVision(String name, Transform3d transformFromRobotOrigin) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transformFromRobotOrigin);
    }

    @Override
    public String getName() {
        return camera.getName();
    }

    @Override
    public Optional<VisionResult> getBotPoseAsVisionResult(boolean allianceFlipped) {
        Optional<EstimatedRobotPose> poseEstimate = poseEstimator.update(camera.getAllUnreadResults().get(0));
        Optional<Pose2d> finalPose = poseEstimate.isPresent()
            ? verifyPose(poseEstimate.get().estimatedPose.toPose2d(), allianceFlipped)
            : Optional.empty();
        if (finalPose.isPresent()) {
            lastPoseEstimate = poseEstimate.get();
            return Optional.of(new VisionResult(finalPose.get(), poseEstimate.get().timestampSeconds));
        }

        return Optional.empty();
    }

    private Optional<Pose2d> verifyPose(Pose2d pose, boolean allianceFlipped) {
        Pose2d measurement = getPose2dAllianceFlipped(pose, allianceFlipped);
        return (measurement.getX() == 0 || measurement.getY() == 0
            ? Optional.empty()
            : (measurement.getTranslation().getDistance(DrivetrainSubsystem.getInstance().getPose().getTranslation()) <= 1
            ? Optional.of(measurement)
            : Optional.empty())
        );
    }

    private Pose2d getPose2dAllianceFlipped(Pose2d pose, boolean toFlip) {
        if (!toFlip || DriverStationHelpers.getAlliance() == Alliance.Blue) return pose;
        return new Pose2d(
            FieldConstants.fieldWidth - pose.getX(),
            FieldConstants.fieldLength - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.k180deg)
        );
    }

    @Override
    public Set<Integer> targets() {
        return lastPoseEstimate.targetsUsed.stream().parallel().map(target -> target.fiducialId).collect(Collectors.toSet());
    }
}
