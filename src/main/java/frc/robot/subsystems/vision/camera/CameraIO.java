package frc.robot.subsystems.vision.camera;

import java.util.Collection;
import java.util.Optional;
import java.util.Set;

import frc.robot.subsystems.vision.VisionResult;

public interface CameraIO {
    String getName();

    /**
     * The Pose Estimation from this camera.
     * <br>
     * @param allianceFlipped whether to flip the pose to match a Blue Alliance origin
     * @return an Optional object that may contain a Pose estimate wrapped in a {@link VisionResult}.
     */
    Optional<VisionResult> getBotPoseAsVisionResult(boolean allianceFlipped);

    /**
     * @return a Collection of all the targets found in the last measurement from this camera.
     */
    Collection<Integer> targets();
}
