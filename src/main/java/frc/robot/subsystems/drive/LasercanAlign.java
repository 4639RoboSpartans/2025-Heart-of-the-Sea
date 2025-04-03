package frc.robot.subsystems.drive;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Robot;
import frc.robot.subsystemManager.SubsystemInstantiator;
import frc.robot.subsystemManager.Subsystems;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.DrivePIDs;

import java.util.OptionalDouble;

import static edu.wpi.first.units.Units.Millimeters;

public class LasercanAlign extends SubsystemBase {
    public static final double targetAlignDistance_mm = 390;

    private static final LinearFilter leftLCFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private static final LinearFilter rightLCFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public static SubsystemInstantiator<LasercanAlign> createInstance() {
        return new SubsystemInstantiator<>(LasercanAlign::new);
    }

    private final LaserCan leftLaserCan, rightLaserCan;
    private final PIDController distanceController;
    private double distance = targetAlignDistance_mm;

    public LasercanAlign() {
        leftLaserCan = new LaserCan(DriveConstants.IDs.LEFT_LASERCAN_ID);
        rightLaserCan = new LaserCan(DriveConstants.IDs.RIGHT_LASERCAN_ID);
        distanceController = new PIDController(
            DrivePIDs.lasercanXkP.get(), 0, 0
        );
        distanceController.setTolerance(0.01);
        distanceController.setSetpoint(targetAlignDistance_mm / 1000);
    }

    private OptionalDouble getMeasurement(LaserCan laserCAN, LinearFilter filter) {
        var measurement = laserCAN.getMeasurement();
        if (measurement == null) {
            return OptionalDouble.empty();
        }
        if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return OptionalDouble.of(
                filter.calculate(
                    measurement.distance_mm
                )
            );
        } else {
            return OptionalDouble.empty();
        }
    }

    public static OptionalDouble getSimMeasurement(boolean left) {
        OptionalDouble distanceFromReefFace = Subsystems.drivetrain().getDistanceFromReefFace();
        if (distanceFromReefFace.isEmpty()) return OptionalDouble.empty();

        Pose2d pose = Subsystems.drivetrain().getPose();
        Pose2d nearestReefPose = DriveCommands.getClosestTarget(() -> pose).transformBy(new Transform2d(0.8, 0, new Rotation2d()));
        Rotation2d rotationDiff = nearestReefPose.getRotation().minus(pose.getRotation());
        double centerDist = distanceFromReefFace.getAsDouble() * 1000;
        double lasercanDistance = DriveConstants.laserCanDistanceMM.in(Millimeters);
        double lasercanCenterDistance = lasercanDistance / 2.0;
        double distanceAdjustment = rotationDiff.getTan() * lasercanCenterDistance;
        double res = (left ? -distanceAdjustment : distanceAdjustment) + centerDist - 573.9;
        if (res >= 1000) return OptionalDouble.empty();
        return OptionalDouble.of(res);
    }

    public OptionalDouble getLeftMeasurement() {
        if (Robot.isSimulation()) return getSimMeasurement(true);
        return getMeasurement(leftLaserCan, leftLCFilter);
    }

    public OptionalDouble getRightMeasurement() {
        if (Robot.isSimulation()) return getSimMeasurement(false);
        return getMeasurement(rightLaserCan, rightLCFilter);
    }

    public OptionalDouble getDistance_mm() {
        OptionalDouble leftMeasurement = getLeftMeasurement();
        OptionalDouble rightMeasurement = getRightMeasurement();

        if (leftMeasurement.isPresent() && rightMeasurement.isPresent()) {
            double average = (leftMeasurement.getAsDouble() + rightMeasurement.getAsDouble()) / 2;
            return OptionalDouble.of(average);
        }
        if (leftMeasurement.isPresent()) {
            return leftMeasurement;
        }
        if (rightMeasurement.isPresent()) {
            return rightMeasurement;
        }
        return OptionalDouble.empty();
    }

    @Override
    public void periodic() {
        distanceController.setP(DrivePIDs.lasercanXkP.get());
        OptionalDouble currentMeasurement = getDistance_mm();
        if (currentMeasurement.isPresent())
            distance = currentMeasurement.getAsDouble();

        distanceController.calculate(distance / 1000);
    }

    public OptionalDouble getOutput() {
        OptionalDouble measuredDistance = getDistance_mm();

        if (measuredDistance.isEmpty()) return OptionalDouble.empty();

        return OptionalDouble.of(
            distanceController.calculate(distance / 1000)
        );
    }
}
