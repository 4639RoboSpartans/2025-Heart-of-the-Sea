package frc.robot.subsystems.drive;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;

import java.util.Objects;

import static edu.wpi.first.units.Units.Millimeters;

public class LasercanAlign extends SubsystemBase {
    private static LasercanAlign instance;
    public static final double alignDistance_mm = 387.5;

    public static LasercanAlign getInstance(SubsystemManager.GetInstanceAccess getInstanceAccess) {
        Objects.requireNonNull(getInstanceAccess);
        return Objects.requireNonNullElseGet(instance, LasercanAlign::new);
    }

    private final LaserCan leftLaserCan, rightLaserCan;
    private final PIDController distanceController;
    private double previousDistance = alignDistance_mm;

    public LasercanAlign() {
        leftLaserCan = new LaserCan(DriveConstants.IDs.LEFT_LASERCAN_ID);
        rightLaserCan = new LaserCan(DriveConstants.IDs.RIGHT_LASERCAN_ID);
        distanceController = new PIDController(
                DrivePIDs.lasercanXkP.get(), 0, 0
        );
        distanceController.setTolerance(0.01);
        distanceController.setSetpoint(alignDistance_mm / 1000);
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
        double res = (left? -distanceAdjustment : distanceAdjustment) + centerDist - 573.9;
        if (res >= 2000) return -1;
        return res;
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
        distanceController.setP(DrivePIDs.lasercanXkP.get());
        double currentMeasurement = getDistance_mm();
        if (currentMeasurement == -1) {
            distanceController.calculate(previousDistance / 1000);
        } else {
            distanceController.calculate(currentMeasurement / 1000);
            previousDistance = currentMeasurement;
        }
    }

    public double getOutput() {
        return distanceController.calculate(getDistance_mm() / 1000);
    }
}
