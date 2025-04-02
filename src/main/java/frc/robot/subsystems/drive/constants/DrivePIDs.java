package frc.robot.subsystems.drive.constants;

import frc.lib.tunable.TunableNumber;

public class DrivePIDs {
    public static TunableNumber pidToPoseXkP = new TunableNumber("Drive PIDs/PID To Pose X Kp");
    public static TunableNumber pidToPoseYkP = new TunableNumber("Drive PIDs/PID To Pose Y Kp");
    public static TunableNumber pidToPosekI = new TunableNumber("Drive PIDs/PID To Pose I");
    public static TunableNumber pidToPosekD = new TunableNumber("Drive PIDs/PID To Pose D");
    public static TunableNumber pidToPoseVelocity = new TunableNumber("Drive PIDs/PID To Pose Velocity");
    public static TunableNumber pidToPoseAcceleration = new TunableNumber("Drive PIDs/PID To Pose Acceleration");
    public static TunableNumber lasercanXkP = new TunableNumber("Drive PIDs/Lasercan PID");

    static {
        pidToPoseXkP.setDefaultValue(3);
        pidToPoseYkP.setDefaultValue(3);
        pidToPoseVelocity.setDefaultValue(2);
        pidToPoseAcceleration.setDefaultValue(1);
        lasercanXkP.setDefaultValue(3);
    }
}
