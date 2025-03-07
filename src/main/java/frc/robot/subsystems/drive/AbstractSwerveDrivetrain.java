package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystems.SubsystemManager;

import java.util.Objects;

public abstract class AbstractSwerveDrivetrain extends SubsystemBase {
    private static AbstractSwerveDrivetrain instance;

    /**
     * This method should only be accessed from the SubsystemManager class. In other places, use
     * {@link SubsystemManager#getDrivetrain()} instead.
     */
    public static AbstractSwerveDrivetrain getInstance(SubsystemManager.GetInstanceAccess access) {
        Objects.requireNonNull(access);

        boolean dummy = false;
        //dummy = true;
        if(dummy) return new DummySwerveDrivetrain();

        return instance = Objects.requireNonNullElseGet(instance,
            Robot.isReal() ? PhysicalSwerveDrivetrain::new : SimSwerveDrivetrain::new
        );
    }

    /**
     * Returns a command that resets the heading of the robot such that the current heading is zero
     *
     * @return Command to run
     */
    public abstract Command resetHeadingToZero();

    /**
     * Returns a command that stops the swerve drive
     *
     * @return Command to run
     */
    public abstract Command stop();

    public abstract void setControl(SwerveRequest control);

    /**
     * Use manual control for the robot using the joysticks
     *
     * @return Command to the run
     */
    public abstract Command manualControl();

    /**
     * Returns a command that moves the robot to the specified pose under PID control, without pathfinding
     *
     * @param targetPose The pose to move to
     * @return Command to run
     */
    public abstract Command directlyMoveTo(Pose2d targetPose);

    /**
     * Returns a command that makes the robot pathfind to the specified pose
     *
     * @param targetPose The pose to move to
     * @return Command to run
     */
    public abstract Command pathfindTo(Pose2d targetPose);

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public abstract void followPath(SwerveSample sample);

    /**
     * Gets the estimated position of the robot as coordinates on the field
     *
     * @return position as Pose2d
     */
    public abstract Pose2d getPose();

    /**
     * Gets the chassis speeds of the robot
     *
     * @return chassis speeds as ChassisSpeeds
     */
    public abstract ChassisSpeeds getChassisSpeeds();

    /**
     * Estimates the acceleration of the robot in gs
     *
     * @return acceleration in gs as double
     */
    public abstract double getAccelerationInGs();

    /**
     * Slows the robot swerve when the elevator is raised. Reduction is proportional to the proportional height of the elevator.
     *
     * @return multiplier as double
     */
    public double getSwerveSpeedMultiplier() {
        return 1 - Math.pow(SubsystemManager.getInstance().getScoringSuperstructure().getElevatorSubsystem().getCurrentExtensionFraction(), 2) * (1.0 - 0.2);
    }

    public abstract void resetPose(Pose2d pose);

    public abstract boolean atTargetPose(Pose2d targetPose);

    public abstract void addVisionMeasurement(Pose2d pose, double timestamp);

    public abstract void setVisionStandardDeviations(double xStdDev, double yStdDev, double rotStdDev);
}
