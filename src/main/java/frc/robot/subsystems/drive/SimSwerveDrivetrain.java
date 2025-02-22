package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SimSwerveDrivetrain extends PhysicalSwerveDrivetrain {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

    public SimSwerveDrivetrain() {
        super();
        // Start the simulation thread
        startSimThread();
    }

    @Override
    public Command directlyMoveTo(Pose2d targetPose) {
        PIDController pidXController = new PIDController(4, 0, 0),
                pidYController = new PIDController(4, 0, 0);
        return new InstantCommand(() -> {
            pidXController.setSetpoint(targetPose.getX());
            pidYController.setSetpoint(targetPose.getY());
        }).andThen(applyRequest(
                () -> {
                    double pidXOutput = pidXController.calculate(getPose().getX());
                    double pidYOutput = pidYController.calculate(getPose().getY());

                    var request = new SwerveRequest.FieldCentricFacingAngle();
                    request.HeadingController = new PhoenixPIDController(8, 0, 0);

                    return request
                            .withVelocityX(pidXOutput)
                            .withVelocityY(pidYOutput)
                            // Michael says not sure why the 180-degree rotation is needed, but it just works
                            .withTargetDirection(targetPose.getRotation().plus(Rotation2d.kZero));
                }
        ).until(
                () -> MathUtil.isNear(targetPose.getX(), getPose().getX(), 0.025)
                        && MathUtil.isNear(targetPose.getY(), getPose().getY(), 0.025)
        ));
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