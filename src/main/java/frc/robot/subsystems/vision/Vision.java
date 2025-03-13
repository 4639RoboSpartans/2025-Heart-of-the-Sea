package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.data.PoseEstimate;
import frc.lib.limelight.data.PoseEstimate.Botpose;
import frc.lib.limelight.data.RawFiducial;
import frc.lib.tunable.TunableNumber;
import frc.lib.util.DriverStationUtil;
import frc.lib.util.PoseUtil;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import com.ctre.phoenix6.Utils;

/**
 * Supporting a global Blue Alliance Coordinate Origin, all input values (which can be either based on Red Alliance or based on Blue Alliance)
 * should be flipped to match what a Blue Alliance robot would see.
 */
public class Vision {
    public static TunableNumber distanceThreshold = new TunableNumber("distanceThresholdMeters").withDefaultValue(1);
    private static final Field2d visionMeasurements = new Field2d();

    static {
        LimelightHelpers.setFiducialIDFiltersOverride("limelight-left", new int[] {6,7,8,9,10,11,17,18,19,20,21,22});
        LimelightHelpers.setFiducialIDFiltersOverride("limelight-right", new int[] {6,7,8,9,10,11,17,18,19,20,21,22});
    }


    public static void addGlobalVisionMeasurements(AbstractSwerveDrivetrain drivetrain, boolean shouldUseMT1STDevs) {
        if ((RobotBase.isReal())) Arrays.stream(Limelights.values()).parallel().forEach(
                limelight -> {
                    Optional<Pose2d> measurement = Optional.of(
                            DriverStationUtil.getAlliance()== DriverStation.Alliance.Blue
                            ? LimelightHelpers.getBotPose2d(limelight.getName(), Botpose.BLUE_MEGATAG1)
                                    : LimelightHelpers.getBotPose2d(limelight.getName(), Botpose.RED_MEGATAG1)
                    );
                    measurement = measurement.get().getX() == 0 && measurement.get().getY() == 0
                            ? Optional.empty()
                            : PoseUtil.withinTolerance(measurement.get(), drivetrain.getPose(), distanceThreshold.get())
                                ? measurement
                                : Optional.empty();

                    measurement.ifPresent(
                        pose -> {
                            if (shouldUseMT1STDevs) {
                                double[] stdevs = LimelightHelpers.getStDevs_MT1(limelight.getName());
                                drivetrain.setVisionStandardDeviations(stdevs[0], stdevs[1], stdevs[3]);
                            }
                            drivetrain.addVisionMeasurement(pose, Utils.getCurrentTimeSeconds());
                        }
                    );
                    SmartDashboard.putNumber("LL Pitch", LimelightHelpers.getBotPose3d(limelight.getName()).getRotation().getMeasureY().baseUnitMagnitude());
                }
        );
    }

    private static Pose2d filterRawMeasurement(PoseEstimate measurement) {
        return measurement.pose().getX() == 0 && measurement.pose().getY() == 0
                ? null
                : measurement.pose();
    }

    private static PoseEstimate getRawMeasurement(Limelights limelight) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.BLUE_MEGATAG1)
                : LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.RED_MEGATAG1);
    }

    public static void filterFiducials(Limelights limelight) {
        PoseEstimate measurement =
                getRawMeasurement(limelight);
        var res = filterRawMeasurement(measurement);

        if (res != null) {
            var rawFiducials = measurement.rawFiducials();
            List<RawFiducial> allFiducials = Arrays.asList(rawFiducials);
            List<Integer> allIds = allFiducials.stream().filter(
                fiducial -> fiducial.distToCamera() >= distanceThreshold.get()
            ).map(RawFiducial::id).toList();

            LimelightHelpers.setFiducialIDFiltersOverride(limelight.getName(), allIds.stream().mapToInt(Integer::intValue).toArray());
        }
    }

    public static OptionalDouble getTA(){
        if (RobotBase.isReal()) return OptionalDouble.of(LimelightHelpers.getTA("limelight"));
        return OptionalDouble.of(Double.MIN_NORMAL);
    }

    public static OptionalDouble getTX(){
        if (RobotBase.isReal()) return OptionalDouble.of(LimelightHelpers.getTX("limelight"));
        return OptionalDouble.of(Double.MIN_NORMAL);
    }
}
