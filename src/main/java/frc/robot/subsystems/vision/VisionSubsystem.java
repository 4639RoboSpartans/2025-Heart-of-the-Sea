package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.subsystems.vision.camera.CameraIO;
import frc.robot.subsystems.vision.camera.LimelightIO;
import frc.robot.subsystems.vision.camera.PhotonVisionIO;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;

public class VisionSubsystem extends SubsystemBase {
    private static volatile VisionSubsystem instance;

    // TODO: why are these HashSets and not Sets? For that matter, why not Collections? Try to use the most general type
    //  that is correct. Also, should cameras be final?
    private HashSet<CameraIO> cameras;
    private HashSet<VisionResult> visionResults;

    public static synchronized VisionSubsystem getInstance() {
        return Objects.requireNonNullElseGet(instance, () -> instance = new VisionSubsystem());
    }

    public VisionSubsystem() {
        cameras = new HashSet<>();
        cameras.addAll(Arrays.stream(IDs.Limelights.values()).map(limelightID -> new LimelightIO(limelightID.getName())).toList());
        cameras.addAll(Arrays.stream(IDs.PhotonCameras.values()).map(photonCameraID -> new PhotonVisionIO(photonCameraID.getName(), photonCameraID.getTranformFromRobotCenter())).toList());
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
            // TODO: Why not just visionResults = new HashSet(), then update? Is there some threading issue? Also, can
            //  use a different approach that fully leverages the Stream API and does not require temporary variable
            //  for the new set created
            //  :
            //      visionResults = cameras.stream().parallel().map(
            //          camera -> camera.getBotPoseAsVisionResult(true)
            //      ).filter(Optional::isPresent).map(Optional::get).collect(Collectors.toSet());
            //  :
            //  Finally, if no outer method will store a reference to visionResults, it could be good to make it final.
            //  In that case, you could do something like
            //  :
            //      visionResults.clear();
            //      cameras.stream().parallel().map(
            //          camera -> camera.getBotPoseAsVisionResult(true)
            //      ).filter(Optional::isPresent).map(Optional::get).forEach(visionResults::add);
            //  :
            //  -- Jonathan

            var _visionResults = new HashSet<VisionResult>();
            cameras.stream().parallel().forEach(
                camera -> camera.getBotPoseAsVisionResult(true).ifPresent(_visionResults::add)
            );
            visionResults = _visionResults;
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
            // TODO: see the comment in globalVision()
            var _visionResults = new HashSet<VisionResult>();
            cameras.stream().parallel().filter(camera -> camera.targets().contains(idToTarget)).forEach(
                camera -> camera.getBotPoseAsVisionResult(true).ifPresent(_visionResults::add)
            );
            visionResults = _visionResults;
        });
    }

    // TODO: again, do other classes need to know that getVisionResults() specifically returns a HashSet, not a Set or
    //  Collection?
    public HashSet<VisionResult> getVisionResults() {
        return visionResults;
    }
}
