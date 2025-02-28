package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.network.LimelightHelpers;
import frc.lib.tunable.TunableNumber;
import frc.lib.util.PoseUtil;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;

public class Vision {
    public static TunableNumber distanceThreshold = new TunableNumber("distanceThresholdMeters").withDefaultValue(1);

    public static void addGlobalVisionMeasurements(AbstractSwerveDrivetrain drivetrain) {
        if ((RobotBase.isReal())) Arrays.stream(Limelights.values()).parallel().forEach(
                limelight -> {
                    Optional<Pose2d> measurement = Optional.of(
                            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                    ? LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName())
                                    : LimelightHelpers.getBotPose2d_wpiRed(limelight.getName())
                    );
                    measurement = measurement.get().getX() == 0 && measurement.get().getY() == 0
                            ? Optional.empty()
                            : PoseUtil.withinTolerance(measurement.get(), drivetrain.getPose(), distanceThreshold.get())
                                ? measurement
                                : Optional.empty();

                    measurement.ifPresent(pose -> drivetrain.addVisionMeasurement(pose, Utils.getCurrentTimeSeconds()));
                }
        );
        getMaximumVisionArea().ifPresent(maximumVisionArea -> SmartDashboard.putNumber("Maximum Vision Area", maximumVisionArea));
        getTX().ifPresent(tx -> SmartDashboard.putNumber("TX", tx));
    }

    public static Optional<Double> getMaximumVisionArea(){
        if (RobotBase.isReal()) return Arrays.stream(Limelights.values()).parallel().map(limelight -> LimelightHelpers.getTA(limelight.getName())).reduce(Math::max);
        return Optional.of(Double.MIN_NORMAL);
    }

    public static OptionalDouble getTX(){
        if (RobotBase.isReal()) return OptionalDouble.of(LimelightHelpers.getTX("limelight"));
        return OptionalDouble.of(Double.MIN_NORMAL);
    }
}
