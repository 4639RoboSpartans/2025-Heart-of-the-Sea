package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.DriverStationHelpers;
import frc.robot.constants.Controls;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class SimSwerveDrivetrain extends PhysicalSwerveDrivetrain {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

    public SimSwerveDrivetrain() {
        super();
        // Start the simulation thread
        startSimThread();
    }

    /**
     * Runs the thread that sets the simulation state
     */
    @SuppressWarnings("resource")
    private void startSimThread() {
        new Notifier(new Runnable() {
            double lastUpdateTimeSeconds = Utils.getCurrentTimeSeconds();

            @Override
            public void run() {
                // Calculate the actual time elapsed since the last invocation
                double deltaTimeSeconds = Utils.getCurrentTimeSeconds() - lastUpdateTimeSeconds;
                lastUpdateTimeSeconds += deltaTimeSeconds;

                drivetrain.updateSimState(
                    deltaTimeSeconds,
                    RobotController.getBatteryVoltage()
                );
            }
        }).startPeriodic(SIM_LOOP_PERIOD); // Run the simulation at a faster rate so PID behaves more reasonably
    }
}