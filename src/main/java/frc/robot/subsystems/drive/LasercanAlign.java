package frc.robot.subsystems.drive;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.constants.DriveConstants;

import java.util.Objects;

import static edu.wpi.first.units.Units.Millimeters;

public class LasercanAlign extends SubsystemBase {
    private static LasercanAlign instance;
    private static final double alignDistance_mm = 500;

    public static LasercanAlign getInstance() {
        return Objects.requireNonNullElseGet(instance, LasercanAlign::new);
    }

    private final LaserCan leftLaserCan, rightLaserCan;
    private final ProfiledPIDController distanceController;
    private double previousDistance = alignDistance_mm;

    public LasercanAlign() {
        leftLaserCan = new LaserCan(DriveConstants.IDs.LEFT_LASERCAN_ID);
        rightLaserCan = new LaserCan(DriveConstants.IDs.RIGHT_LASERCAN_ID);
        distanceController = new ProfiledPIDController(
                1, 0, 0,
                new TrapezoidProfile.Constraints(
                        10, 10
                )
        );
        distanceController.setGoal(alignDistance_mm);
    }

    private double getMeasurement(LaserCan laserCAN) {
        var measurement = laserCAN.getMeasurement();
        if (measurement == null) {
            return -1;
        }
        if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } else {
            return -1;
        }
    }

    public static double getSimMeasurement(boolean left) {
        Pose2d pose = SubsystemManager.getInstance().getDrivetrain().getPose();
        Pose2d nearestReefPose = DriveCommands.getClosestTarget(() -> pose).transformBy(new Transform2d(0.8, 0, new Rotation2d()));
        Rotation2d rotationDiff = nearestReefPose.getRotation().minus(pose.getRotation());
        double centerDist = SubsystemManager.getInstance().getDrivetrain().getDistanceFromReefFace() * 1000;
        double lasercanDistance = DriveConstants.laserCanDistanceMM.in(Millimeters);
        double lasercanCenterDistance = lasercanDistance / 2.0;
        double distanceAdjustment = rotationDiff.getTan() * lasercanCenterDistance;
        return (left? -distanceAdjustment : distanceAdjustment) + centerDist + 150;
    }

    public double getLeftMeasurement() {
        if (Robot.isSimulation()) return getSimMeasurement(true);
        return getMeasurement(leftLaserCan);
    }

    public double getRightMeasurement() {
        if (Robot.isSimulation()) return getSimMeasurement(false);
        return getMeasurement(rightLaserCan);
    }

    public double getDistance_mm() {
        double leftMeasurement = getLeftMeasurement();
        double rightMeasurement = getRightMeasurement();
        if (leftMeasurement == -1) {
            return rightMeasurement;
        } else {
            if (rightMeasurement == -1) {
                return leftMeasurement;
            } else {
                return (getLeftMeasurement() + getRightMeasurement()) / 2.0;
            }
        }
    }

    @Override
    public void periodic() {
        double currentMeasurement = getDistance_mm();
        if (currentMeasurement == -1) {
            distanceController.calculate(previousDistance);
        } else {
            distanceController.calculate(currentMeasurement);
            previousDistance = currentMeasurement;
        }
    }

    public double getOutput() {
        return distanceController.calculate(getDistance_mm());
    }
}
