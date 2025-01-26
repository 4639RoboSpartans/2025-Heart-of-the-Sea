package frc.robot.vision;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.network.LimelightHelpers;
import frc.robot.constants.IDs;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class VisionIO {
    /** Checks every Limelight Camera for pose estimates and adds them to the Kalman filter. 
     * <p><b>drivetrain</b>: the drivetrain to add pose estimates to.</p>
     * 
     * <p><b>distanceThreshold</b>: the maximum distance in meters away from the extant Robot pose for a vision measurement to be considered.</p>
    */
    public static void addGlobalVisionMeasurementsToDriveTrain(CommandSwerveDrivetrain drivetrain, double distanceThreshold){
        if (RobotBase.isReal()) Arrays.stream(IDs.Limelights.values()).forEach(
            limelight -> {
                Optional<Pose2d> measurement = Optional.of(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName())
                        : LimelightHelpers.getBotPose2d_wpiRed(limelight.getName())
                );
                measurement = measurement.isPresent()
                                ? (measurement.get().getX() == 0 || measurement.get().getY() == 0
                                    ? Optional.empty()
                                    : (measurement.get().getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()) <= distanceThreshold
                                        ? measurement
                                        : Optional.empty())
                                    )
                                : Optional.empty();
                measurement.ifPresent(pose -> drivetrain.addVisionMeasurement(pose, Utils.getCurrentTimeSeconds()));
            }
        );
    }

    //I want to implement something specific for lining up to a target here in the future
}
