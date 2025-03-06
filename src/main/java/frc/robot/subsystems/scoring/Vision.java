package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.data.PoseEstimate;
import frc.lib.limelight.data.PoseEstimate.Botpose;
import frc.lib.tunable.TunableNumber;
import frc.lib.util.PoseUtil;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.Arrays;
import java.util.OptionalDouble;

public class Vision {
    public static TunableNumber distanceThreshold = new TunableNumber("distanceThresholdMeters").withDefaultValue(1);

    public static void addGlobalVisionMeasurements(AbstractSwerveDrivetrain drivetrain) {
        if ((RobotBase.isReal())) Arrays.stream(Limelights.values()).parallel().forEach(
                limelight -> {
                    PoseEstimate measurement = 
                            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                    ? LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.BLUE_MEGATAG1)
                                    : LimelightHelpers.getBotPoseEstimate(limelight.getName(), Botpose.RED_MEGATAG1);
                    var res = measurement.pose().getX() == 0 && measurement.pose().getY() == 0
                            ? null
                            : PoseUtil.withinTolerance(measurement.pose(), drivetrain.getPose(), distanceThreshold.get())
                                ? measurement.pose()
                                : null;

                    if (res != null) {
                        double[] stdevs = LimelightHelpers.getStDevs_MT1(limelight.getName());
                        drivetrain.setVisionStandardDeviations(stdevs[0], stdevs[1], stdevs[3]);
                        drivetrain.addVisionMeasurement(res, measurement.timestampSeconds());
                    }
                }
        );
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
