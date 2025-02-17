package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.annotation.ForSubsystemManagerUseOnly;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.subsystems.vision.camera.Limelight;
import frc.robot.subsystems.vision.camera.PhotonVision;
import frc.robot.subsystems.vision.constants.Limelights;
import frc.robot.subsystems.vision.constants.PhotonCameras;

import java.util.*;

public class VisionSubsystem extends SubsystemBase {
    private static volatile VisionSubsystem instance;

    private final Set<Camera> cameras;
    private HashSet<VisionResult> visionResults;

    /**
     * This method should only be accessed from the SubsystemManager class. In other places, use
     * {@link SubsystemManager#getVisionSubsystem()} instead.
     */
    @ForSubsystemManagerUseOnly
    public static synchronized VisionSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, VisionSubsystem::new);
    }

    public VisionSubsystem() {
        cameras = new HashSet<>();
        visionResults = new HashSet<>();
        cameras.addAll(Arrays.stream(Limelights.values()).map(limelightID -> new Limelight(limelightID.getName())).toList());
        cameras.addAll(Arrays.stream(PhotonCameras.values()).map(photonCameraID -> new PhotonVision(photonCameraID.getName(), photonCameraID.getTranformFromRobotCenter())).toList());
    }

    @Override
    public void periodic() {
        if (this.getCurrentCommand() == null) visionResults = new HashSet<>();
    }

    /**
     * Pushes valid vision measurements from all cameras and all april tags.
     */
    public Command globalVision() {
        return run(() -> {
            visionResults.clear();
            cameras.stream().parallel().map(
                    camera -> camera.getBotPoseAsVisionResult(true)
            ).filter(Optional::isPresent).map(Optional::get).forEach(visionResults::add);
        });
    }

    /**
     * Pushes valid vision measurements from only the cameras that see a specific april tag.
     * <p>
     * Useful for precise targeting when the robot will be near a specific april tag and it is beneficial to eliminate other, potentially less precise measurements.
     *
     * @param idToTarget april tag ID to target
     */
    public Command targetedVision(int idToTarget) {
        return run(() -> {
            visionResults.clear();
            cameras.stream().parallel().filter(camera -> camera.targets().contains(idToTarget))
                    .map(camera -> camera.getBotPoseAsVisionResult(true))
                    .filter(Optional::isPresent).map(Optional::get).forEach(visionResults::add);
        });
    }

    /**
     * @param idToTarget AprilTagIDHolder representing a position on the field irrespective of alliance
     * @return {@link VisionSubsystem#targetedVision(int)} using the AprilTag ID consistent with the current alliance
     */
    public Command targetedVision(FieldConstants.AprilTagIDHolder idToTarget) {
        return targetedVision(idToTarget.getAllianceRespectiveID());
    }

    public Set<VisionResult> getVisionResults() {
        return visionResults;
    }
}
