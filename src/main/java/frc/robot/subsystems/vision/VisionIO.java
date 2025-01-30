package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.network.LimelightHelpers;
import frc.robot.constants.Limelights;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class VisionIO extends SubsystemBase{
    private static boolean useVision = false;
    private static boolean trustFaraways = false;

    private static int defaultTargetID = -1;

    private static Runnable visionFunction = VisionIO::addGlobalVisionMeasurementsToDriveTrain;

    static {
        SmartDashboard.putBoolean("Trust Faraway Data", trustFaraways);
    }

    /** Checks every Limelight Camera for pose estimates and adds them to the Kalman filter. 
     * @param drivetrain the drivetrain to add pose estimates to.</p>
     * 
     * @param distanceThreshold the maximum distance in meters away from the extant Robot pose for a vision measurement to be considered.</p>
    */
    public static void addGlobalVisionMeasurementsToDriveTrain(CommandSwerveDrivetrain drivetrain, double distanceThreshold){
        trustFaraways = SmartDashboard.getBoolean("Trust Faraway Data", trustFaraways);
        if ((useVision || trustFaraways) && RobotBase.isReal()) Arrays.stream(Limelights.values()).parallel().forEach(
            limelight -> {
                Optional<Pose2d> measurement = Optional.of(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName())
                        : LimelightHelpers.getBotPose2d_wpiRed(limelight.getName())
                );
                measurement = measurement.isPresent()
                                ? (measurement.get().getX() == 0 || measurement.get().getY() == 0
                                    ? Optional.empty()
                                    : (measurement.get().getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()) <= (trustFaraways ? 1000 : distanceThreshold)
                                        ? measurement
                                        : Optional.empty())
                                    )
                                : Optional.empty();
                measurement.ifPresent(pose -> drivetrain.addVisionMeasurement(pose, Utils.getCurrentTimeSeconds()));
            }
        );
    }

    public static void target(CommandSwerveDrivetrain drivetrain, int ID){
        if(useVision && RobotBase.isReal()) Arrays.stream(Limelights.values()).parallel().forEach(
            limelight -> {
                if(LimelightHelpers.getRawDetections(limelight.getName()).length == 1
                    && 
                Arrays.stream(LimelightHelpers.getRawDetections(limelight.getName())).parallel().filter(detection -> detection.classId == ID).findAny().isPresent())
                {
                    Optional<Pose2d> measurement = Optional.of(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? LimelightHelpers.getBotPose2d_wpiBlue(limelight.getName())
                        : LimelightHelpers.getBotPose2d_wpiRed(limelight.getName())
                    );
                    measurement = measurement.isPresent()
                                ? (measurement.get().getX() == 0 || measurement.get().getY() == 0
                                    ? Optional.empty()
                                    : (measurement.get().getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()) <= 1
                                        ? measurement
                                        : Optional.empty())
                                    )
                                : Optional.empty();
                    measurement.ifPresent(pose -> drivetrain.addVisionMeasurement(pose, Utils.getCurrentTimeSeconds()));
                }
            }
        );
    }

    public static void target(){
        target(CommandSwerveDrivetrain.getInstance(), defaultTargetID);
    }

    public static void addGlobalVisionMeasurementsToDriveTrain(){
        addGlobalVisionMeasurementsToDriveTrain(CommandSwerveDrivetrain.getInstance(), 1.0);
    }

    public static InstantCommand setTargetID(int ID){
        return new InstantCommand(() -> defaultTargetID = ID, new Subsystem[0]);
    }

    public static InstantCommand setVisionFunction(Runnable r){
        return new InstantCommand(() -> visionFunction = r, new Subsystem[0]);
    }

    public static Runnable getVisionFunction(){
        return visionFunction;
    }
}
