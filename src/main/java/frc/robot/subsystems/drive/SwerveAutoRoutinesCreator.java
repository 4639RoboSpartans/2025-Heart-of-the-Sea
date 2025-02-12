package frc.robot.subsystems.drive;

import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.commands.AutoRoutines;

public final class SwerveAutoRoutinesCreator {
    private static final PIDConstants AUTON_TRANSLATION_PID_CONSTANTS = new PIDConstants(10, 0, 0);
    private static final PIDConstants AUTON_ROTATION_PID_CONSTANTS = new PIDConstants(7, 0, 0);

    public static AutoRoutines createAutoRoutines(CommandSwerveDrivetrain drivetrain) {
        RobotConfig config = RobotConfigLoader.getOrLoadConfig();

        AutoBuilder.configure(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::getChassisSpeeds,
            // Function that uses chassisSpeeds and feedforward values to drive the robot
            (speeds, feedforwards) -> drivetrain.setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),

            new PPHolonomicDriveController(
                AUTON_TRANSLATION_PID_CONSTANTS,
                AUTON_ROTATION_PID_CONSTANTS
            ),
            config,
            // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            () -> false,
            drivetrain // Subsystem for requirements
        );

        return new AutoRoutines(new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::followPath,
            true,
            drivetrain,
            (sample, isStart) -> {}
        ));
    }
}
