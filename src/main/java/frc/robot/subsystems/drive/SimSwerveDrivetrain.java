package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimSwerveDrivetrain extends PhysicalSwerveDrivetrain {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

    public SimSwerveDrivetrain() {
        super();
        // Start the simulation thread
        startSimThread();
    }

    @Override
    public double getDistanceFromReefFace() {
        Pose2d nearestReefPose = DriveCommands.getClosestTarget(this::getPose).transformBy(new Transform2d(0.8, 0, new Rotation2d()));
        super.getField().getObject("nearest reef pose").setPose(nearestReefPose);
        Translation2d nearestReefTranslation = nearestReefPose.getTranslation();
        Rotation2d reefRotation = nearestReefPose.getRotation();
        Translation2d robotTranslation = getPose().getTranslation();
        Vector<N2> robotOffsetFromReefFace = robotTranslation.minus(nearestReefTranslation).toVector();
        Vector<N2> reefNormalVector = VecBuilder.fill(reefRotation.getCos(), reefRotation.getSin());
        return -robotOffsetFromReefFace.dot(reefNormalVector);
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

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Distance to Closest Reef", getDistanceFromReefFace());
        SmartDashboard.putNumber("Left LaserCAN Distance", LasercanAlign.getSimMeasurement(true));
        SmartDashboard.putNumber("Right LaserCAN Distance", LasercanAlign.getSimMeasurement(false));
    }
}