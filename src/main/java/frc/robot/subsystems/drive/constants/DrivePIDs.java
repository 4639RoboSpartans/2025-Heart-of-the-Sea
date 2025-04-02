package frc.robot.subsystems.drive.constants;

import frc.lib.tunable.TunableNumber;

public class DrivePIDs {
    public static TunableNumber pidToPoseXkP = new TunableNumber("Drive PIDs/PID To Pose X Kp");
    public static TunableNumber pidToPoseYkP = new TunableNumber("Drive PIDs/PID To Pose Y Kp");
    public static TunableNumber pidToPoseVelocityX = new TunableNumber("Drive PIDs/PID To Pose VelocityX");
    public static TunableNumber pidToPoseAccelerationX = new TunableNumber("Drive PIDs/PID To Pose AccelerationX");
    public static TunableNumber pidToPoseVelocityY = new TunableNumber("Drive PIDs/PID To Pose VelocityY");
    public static TunableNumber pidToPoseAccelerationY = new TunableNumber("Drive PIDs/PID To Pose AccelerationY");
    public static TunableNumber lasercanXkP = new TunableNumber("Drive PIDs/Lasercan PID");

    static {
        pidToPoseXkP.setDefaultValue(2);
        pidToPoseYkP.setDefaultValue(6);
        pidToPoseVelocityX.setDefaultValue(2);
        pidToPoseAccelerationX.setDefaultValue(1);
        pidToPoseVelocityX.setDefaultValue(4);
        pidToPoseAccelerationX.setDefaultValue(2);
        lasercanXkP.setDefaultValue(3);
    }
}
