package frc.robot.subsystems.drive.constants;

import frc.lib.tunable.TunableNumber;

public class DrivePIDs {
    public static TunableNumber pidToPoseXkP = new TunableNumber("Drive PIDs/PID To Pose X Kp");
    public static TunableNumber pidToPoseYkP = new TunableNumber("Drive PIDs/PID To Pose Y Kp");
    public static TunableNumber pidToPoseVelocity = new TunableNumber("Drive PIDs/PID To Pose Velocity");
    public static TunableNumber pidToPoseAcceleration = new TunableNumber("Drive PIDs/PID To Pose Acceleration");

    static {
        pidToPoseXkP.setDefaultValue(60);
        pidToPoseYkP.setDefaultValue(60);
        pidToPoseVelocity.setDefaultValue(2);
        pidToPoseAcceleration.setDefaultValue(1);
    }
}
