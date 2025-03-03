package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.network.LimelightHelpers;
import frc.lib.tunable.TunableNumber;
import frc.lib.util.PoseUtil;
import frc.robot.subsystems.drive.AbstractSwerveDrivetrain;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

public class Vision {
    public static TunableNumber distanceThreshold = new TunableNumber("distanceThresholdMeters").withDefaultValue(1);

    public static void addGlobalVisionMeasurements(AbstractSwerveDrivetrain drivetrain) {
        if (RobotBase.isReal()) Limelights.all.parallelStream().map(limelight -> Pair.of(limelight.poseEstimateMegaTag1.get().pose, limelight.poseEstimateMegaTag1.get().timestampSeconds)).forEach(pair -> drivetrain.addVisionMeasurement(pair.getFirst(), pair.getSecond()));
    }
}
