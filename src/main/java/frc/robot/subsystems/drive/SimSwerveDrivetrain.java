package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.DriverStationUtil;
import frc.lib.util.PoseUtil;

public class SimSwerveDrivetrain extends PhysicalSwerveDrivetrain {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

    public SimSwerveDrivetrain() {
        super();
        // Start the simulation thread
        startSimThread();
    }

    @Override
    public Command directlyMoveTo(Pose2d targetPose) {
        return new InstantCommand(() -> {
            pidXController.reset(getPose().getX());
            pidYController.reset(getPose().getY());
            pidXController.setGoal(targetPose.getX());
            pidYController.setGoal(targetPose.getY());
        }).andThen(applyRequest(
                () -> {
                    field.getObject("Target Pose").setPose(pidXController.getSetpoint().position, pidYController.getSetpoint().position, targetPose.getRotation());
                    double directionMultiplier = DriverStationUtil.getAlliance() == DriverStation.Alliance.Red? -1 : 1;
                    double pidXOutput = pidXController.calculate(getPose().getX()) * directionMultiplier;
                    double pidYOutput = pidYController.calculate(getPose().getY()) * directionMultiplier;

                    var request = new SwerveRequest.FieldCentricFacingAngle();
                    request.HeadingController = new PhoenixPIDController(8, 0, 0);
                    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
                    Rotation2d headingOffset = DriverStationUtil.getAlliance() == DriverStation.Alliance.Red? Rotation2d.k180deg : new Rotation2d();

                    return request
                            .withVelocityX(pidXOutput)
                            .withVelocityY(pidYOutput)
                            .withTargetDirection(targetPose.getRotation().plus(headingOffset));
                }
        ).until(
                () -> PoseUtil.withinTolerance(targetPose, getPose(), Units.inchesToMeters(2))
                        && MathUtil.isNear(targetPose.getRotation().getDegrees(), getPose().getRotation().getDegrees(), 2)
        )).andThen(stop().withTimeout(0.1));
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