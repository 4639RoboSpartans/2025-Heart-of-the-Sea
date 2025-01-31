package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.subsystems.vision.camera.CameraIO;
import frc.robot.subsystems.vision.camera.LimelightIO;
import frc.robot.subsystems.vision.camera.PhotonVisionIO;

public class VisionSubsystem extends SubsystemBase{
    private static volatile VisionSubsystem instance;

    private HashSet<CameraIO> cameras;
    private HashSet<VisionResult> visionResults;

    public static synchronized VisionSubsystem getInstance(){
        return Objects.requireNonNullElseGet(instance, () -> instance = new VisionSubsystem());
    }

    public VisionSubsystem(){
        cameras = new HashSet<CameraIO>();
        cameras.addAll(Arrays.stream(IDs.Limelights.values()).map(limelightID -> new LimelightIO(limelightID.getName())).toList());
        cameras.addAll(Arrays.stream(IDs.PhotonCameras.values()).map(photonCameraID -> new PhotonVisionIO(photonCameraID.getName(), photonCameraID.getTranformFromRobotCenter())).toList());
    }

    @Override
    public void periodic(){
        if(this.getCurrentCommand() == null) visionResults = new HashSet<>();
    }

    /**
     * Pushes valid vision measurements from all cameras and all april tags.
     */
    public Command globalVision(){
        return run(() -> {
            var _visionResults = new HashSet<VisionResult>();
            cameras.stream().parallel().forEach(
            camera -> camera.getBotPoseAsVisionResult(true).ifPresent(visionResult -> 
                _visionResults.add(visionResult)
            )
            );
            visionResults = _visionResults;
        });
    }

    /**
     * Pushes valid vision measurements from only the cameras that see a specific april tag.
     * <p>
     * Useful for precise targeting when the robot will be near a specific april tag and it is beneficial to eliminate other, potentially less precise measurements.
     * @param idToTarget april tag ID to target
     */
    public Command targetedVision(int idToTarget){
        return run(() -> {
            var _visionResults = new HashSet<VisionResult>();
            cameras.stream().parallel().filter(camera -> camera.targets().contains(idToTarget)).forEach(
            camera -> camera.getBotPoseAsVisionResult(true).ifPresent(visionResult -> 
                _visionResults.add(visionResult)
            )
            );
            visionResults = _visionResults;
        });
    }

    public HashSet<VisionResult> getVisionResults(){
        return visionResults;
    }
}
