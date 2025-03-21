package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveConstants {
    public static final double centerToWheel = 0.245;

    // Change MOVEMENT_SPEED to 1.0 for max speed
    public static final double CURRENT_MAX_ROBOT_MPS = 3.5;
    public static final double TELOP_ROTATION_SPEED = 12;


    public static final PIDController choreoX = new PIDController(7, 0, 0.1);
    public static final PIDController choreoY = new PIDController(7, 0, 0.1);
    public static final PIDController choreoRotation = new PIDController(4, 0, 0.1);
    public static final double TIME_BEFORE_INTAKE_START = 1;

    public static final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.kZero;
    public static final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.k180deg;

    public static final double leftTargetTA = 6.79; //TODO: tune these
    public static final double leftTargetTX = -0.47;

    public static final double rightTargetTA = 0.0;
    public static final double rightTargetTX = 0.0;

    public static final double TXTolerance = 0.01;
    public static final double TATolerance = 0.01;
}
