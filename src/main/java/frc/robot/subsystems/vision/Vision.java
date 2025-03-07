package frc.robot.subsystems.vision;

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
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;

import com.ctre.phoenix6.Utils;

public class Vision {
    public static TunableNumber distanceThreshold = new TunableNumber("distanceThresholdMeters").withDefaultValue(1);
    private static Field2d visionMeasurements = new Field2d();

    static {
        LimelightHelpers.setFiducialIDFiltersOverride("limelight-left", FieldConstants.kReefAprilTags);
        LimelightHelpers.setFiducialIDFiltersOverride("limelight-right", FieldConstants.kReefAprilTags);
    }

    public static void addGlobalVisionMeasurements(AbstractSwerveDrivetrain drivetrain) {
        if ((RobotBase.isReal())) {
            Arrays.stream(Limelights.values()).parallel().forEach(
                limelight -> {
                    if (RobotState.isAutonomous()) {
                        filterFiducials(limelight);
                    } else {
                        LimelightHelpers.setFiducialIDFiltersOverride(
                            limelight.getName(), 
                            new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22}
                        );
                    }
                    PoseEstimate measurement = 
                            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                    ? LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.BLUE_MEGATAG1)
                                    : LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.RED_MEGATAG1);
                    var res = measurement.pose().getX() == 0 && measurement.pose().getY() == 0
                            ? null
                            : measurement.pose();

                    if (res != null) {
                        double[] stdevs = LimelightHelpers.getStDevs_MT1(limelight.getName());
                        if (RobotState.isAutonomous()) {
                            drivetrain.setVisionStandardDeviations(10, 10, 999999);
                        } else if (RobotState.isDisabled()) {
                            drivetrain.setVisionStandardDeviations(stdevs[0], stdevs[1], stdevs[3]);
                        } else {
                            drivetrain.setVisionStandardDeviations(stdevs[0] * 100, stdevs[1] * 100, stdevs[3]);
                        }
                        drivetrain.addVisionMeasurement(res, Utils.getCurrentTimeSeconds());
                        visionMeasurements.getObject(limelight.getName()).setPose(res);
                        SmartDashboard.putBoolean("Added Vision", true);
                    } else {
                        SmartDashboard.putBoolean("Added Vision", false);
                    }
                    SmartDashboard.putData("Vision Measurements", visionMeasurements);
                }
            );
        }
    }

    public static void filterFiducials(Limelights limelight) {
        PoseEstimate measurement = 
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                        ? LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.BLUE_MEGATAG1)
                        : LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.RED_MEGATAG1);
        var res = measurement.pose().getX() == 0 && measurement.pose().getY() == 0
                ? null
                : measurement.pose();

        if (res != null) {
            var rawFiducials = measurement.rawFiducials();
            List<RawFiducial> allFiducials = new ArrayList<>();
            for (RawFiducial fiducial : rawFiducials) allFiducials.add(fiducial);
            List<Integer> allIds = allFiducials.stream().filter(
                fiducial -> fiducial.distToCamera() >= distanceThreshold.get()
            ).map(RawFiducial::id).toList();
            int[] allIdInts = new int[allIds.size()];
            for (int i = 0; i < allIds.size(); i++) {
                allIdInts[i] = allIds.get(i);
            }
            LimelightHelpers.setFiducialIDFiltersOverride(limelight.getName(), allIdInts);
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
