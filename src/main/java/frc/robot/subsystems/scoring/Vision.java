package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.network.LimelightHelpers;
import frc.lib.tunable.TunableNumber;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.Arrays;
import java.util.Optional;

public class Vision {
    public static TunableNumber distanceThreshold = new TunableNumber("distanceThresholdMeters").withDefaultValue(1);

    public static void addGlobalVisionMeasurements(AbstractSwerveDrivetrain drivetrain) {
        if ((RobotBase.isReal())) Arrays.stream(Limelights.values()).parallel().forEach(
                limelight -> {
                    Optional<Pose2d> measurement = Optional.of(
                            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                    ? LimelightHelpers.getBotPose2d_wpiBlue(limelight.toString())
                                    : LimelightHelpers.getBotPose2d_wpiRed(limelight.toString())
                    );
                    measurement = measurement.isPresent()
                            ? (measurement.get().getX() == 0 || measurement.get().getY() == 0
                            ? Optional.empty()
                            : (measurement.get().getTranslation().getDistance(drivetrain.getPose().getTranslation()) <= (distanceThreshold.get())
                            ? measurement
                            : Optional.empty())
                    )
                            : Optional.empty();
                    measurement.ifPresent(pose -> drivetrain.addVisionMeasurement(pose, Utils.getCurrentTimeSeconds()));
                }
        );
    }
}
