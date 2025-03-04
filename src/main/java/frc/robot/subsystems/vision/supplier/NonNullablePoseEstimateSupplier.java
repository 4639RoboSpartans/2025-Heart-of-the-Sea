package frc.robot.subsystems.vision.supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.network.LimelightHelpers;

import java.util.Optional;
import java.util.function.Supplier;

public class NonNullablePoseEstimateSupplier implements Supplier<LimelightHelpers.PoseEstimate> {
    Supplier<LimelightHelpers.PoseEstimate> rawSupplier;
    LimelightHelpers.PoseEstimate lastNonNullMeasurement = new LimelightHelpers.PoseEstimate(new Pose2d(), 0.0, 0.0, 0, 0.0, 0.0, 0.0, new LimelightHelpers.RawFiducial[]{}, false);
    public NonNullablePoseEstimateSupplier(Supplier<LimelightHelpers.PoseEstimate> rawSupplier) {
        this.rawSupplier = rawSupplier;
    }

    /**
     * Gets a result.
     *
     * @return a result
     */
    @Override
    public LimelightHelpers.PoseEstimate get() {
        try {
            lastNonNullMeasurement = Optional.ofNullable(rawSupplier.get()).orElse(lastNonNullMeasurement);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return lastNonNullMeasurement;
    }
}
