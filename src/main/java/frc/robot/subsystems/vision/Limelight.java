package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.network.LimelightHelpers;
import frc.lib.util.PoseUtil;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;
import frc.robot.subsystems.vision.supplier.SuppyGyroPoseUnlessTargetInSight;
import frc.robot.subsystems.vision.supplier.NonNullablePoseEstimateSupplier;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * Wraps a limelight
 */
public class Limelight {
    private String name;

    private static AbstractSwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();

    public Limelight(String name) {
        this.name = name;
    }

    /**
     * Use this for general limelight pose
     */
    public LimelightHelpers.PoseEstimate getCurrentPoseEstimateMegaTag1(){
        if (!RobotBase.isReal()) return null;
        LimelightHelpers.PoseEstimate measurement =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue(getName())
                        : LimelightHelpers.getBotPoseEstimate_wpiRed(getName());
        return measurement.pose.getX() == 0 && measurement.pose.getY() == 0
                ? null
                : (PoseUtil.withinTolerance(measurement.pose, drivetrain.getPose(), Vision.distanceThreshold.get())
                ? measurement
                : null);
    }

    /**
     * use this for precise localization maybe??
     */
    private LimelightHelpers.PoseEstimate getCurrentPoseEstimateMegaTag2(){
        if (!RobotBase.isReal()) return null;
        LimelightHelpers.SetRobotOrientation(getName(), drivetrain.getPose().getRotation().getDegrees(), Rotation2d.fromRadians(drivetrain.getChassisSpeeds().omegaRadiansPerSecond).getDegrees(), 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate measurement =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getName())
                        : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(getName());
        return measurement.pose.getX() == 0 && measurement.pose.getY() == 0
                ? null
                : PoseUtil.withinTolerance(measurement.pose, drivetrain.getPose(), Vision.distanceThreshold.get())
                ? measurement
                : null;
    }

    public boolean hasTarget(int fiducialID){
        return Optional.ofNullable(getCurrentPoseEstimateMegaTag1()).map(poseEstimate -> Arrays.stream(poseEstimate.rawFiducials).parallel().anyMatch(fiducial -> fiducial.id == fiducialID)).orElse(false);
    }

    public boolean onlyHasTarget(int fiducialID){
        return hasTarget(fiducialID) && Optional.ofNullable(getCurrentPoseEstimateMegaTag1()).map(poseEstimate -> List.of(poseEstimate.rawFiducials)).orElse(new ArrayList<>()).size() == 1;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Supplier<LimelightHelpers.PoseEstimate> poseEstimateMegaTag1 = new NonNullablePoseEstimateSupplier(this::getCurrentPoseEstimateMegaTag1);
    public Supplier<LimelightHelpers.PoseEstimate> poseEstimateMegaTag2 = new NonNullablePoseEstimateSupplier(this::getCurrentPoseEstimateMegaTag2);

    public Supplier<Pose2d> createVisionAlignPoseSupplier(FieldConstants.AprilTagIDHolder tagIDHolder){
        return new SuppyGyroPoseUnlessTargetInSight(this::getCurrentPoseEstimateMegaTag2, () -> hasTarget(tagIDHolder.getAllianceRespectiveID()));
    }

    public void filterRawFiducials(int... fiducialIDs){
        LimelightHelpers.SetFiducialIDFiltersOverride(this.getName(), fiducialIDs);
    }

    public void resetFiducialFilter(){
        LimelightHelpers.SetFiducialIDFiltersOverride(this.getName(), Arrays.stream(FieldConstants.AprilTagIDHolder.values()).parallel().flatMapToInt(IDHolder -> IntStream.of(IDHolder.getBlueAllianceID(), IDHolder.getRedAllianceID())).toArray());
    }
}
