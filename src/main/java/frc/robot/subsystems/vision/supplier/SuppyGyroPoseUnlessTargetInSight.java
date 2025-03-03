package frc.robot.subsystems.vision.supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.network.LimelightHelpers;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SuppyGyroPoseUnlessTargetInSight implements Supplier<Pose2d> {
    private AbstractSwerveDrivetrain drivetrain;
    private Supplier<LimelightHelpers.PoseEstimate> limelightPoseEstimateSupplier;
    private BooleanSupplier hasTargetSupplier;

    public SuppyGyroPoseUnlessTargetInSight(Supplier<LimelightHelpers.PoseEstimate> limelightPoseEstimateSupplier, BooleanSupplier hasTargetSupplier) {
        this.limelightPoseEstimateSupplier = limelightPoseEstimateSupplier;
        this.hasTargetSupplier = hasTargetSupplier;
    }

    /**
     * Gets a result.
     *
     * @return a result
     */
    @Override
    public Pose2d get() {
        return hasTargetSupplier.getAsBoolean() ? limelightPoseEstimateSupplier.get().pose : drivetrain.getPose();
    }
}
