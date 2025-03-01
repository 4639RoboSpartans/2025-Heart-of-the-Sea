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
                                    ? LimelightHelpers.getBotPose2d_wpiRed(limelight.getName())
                                    : LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName())
                    );
                    measurement = measurement.get().getX() == 0 && measurement.get().getY() == 0
                            ? Optional.empty()
                            : PoseUtil.withinTolerance(measurement.get(), drivetrain.getPose(), distanceThreshold.get())
                                ? measurement
                                : Optional.empty();

                    measurement.ifPresent(pose -> {
                        drivetrain.getField().getObject("Vision Measurement "+ limelight.getName()).setPose(pose);
                        drivetrain.addVisionMeasurement(pose, Utils.getCurrentTimeSeconds());
                    });
                }
        );
        if (RobotBase.isReal()){
            postTAs();
            postTXs();
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

    public static void postTAs(){
        SmartDashboard.putNumberArray("limelight", LimelightHelpers.getTargetPose_CameraSpace("limelight"));
        SmartDashboard.putNumberArray("limelight-slhs", LimelightHelpers.getTargetPose_CameraSpace("limelight-slhs"));
        SmartDashboard.putNumber("Right TZ", LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);
        SmartDashboard.putNumber("Left TZ", LimelightHelpers.getTargetPose_CameraSpace("limelight-slhs")[2]);
    }

    public static void postTXs(){
        SmartDashboard.putNumber("Right TY", LimelightHelpers.getTargetPose_CameraSpace("limelight-slhs")[1]);
        SmartDashboard.putNumber("Left TY", LimelightHelpers.getTargetPose_CameraSpace("limelight-slhs")[1]);
        SmartDashboard.putNumber("Right TX", LimelightHelpers.getTargetPose_CameraSpace("limelight-slhs")[0]);
        SmartDashboard.putNumber("Left TX", LimelightHelpers.getTargetPose_CameraSpace("limelight-slhs")[0]);
    }
}
