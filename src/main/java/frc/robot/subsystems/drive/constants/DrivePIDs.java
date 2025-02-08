package frc.robot.subsystems.drive.constants;

import frc.lib.TunableNumber;

public class DrivePIDs {
    public static TunableNumber pidToPoseXkP = new TunableNumber("PID To Pose X Kp");
    public static TunableNumber pidToPoseYkP = new TunableNumber("PID To Pose Y Kp");
    public static TunableNumber pidToPoseVelocity = new TunableNumber("PID To Pose Velocity");
    public static TunableNumber pidToPoseAcceleration = new TunableNumber("PID To Pose Acceleration");

    static {
        pidToPoseXkP.setDefault(5);
        pidToPoseYkP.setDefault(5);
        pidToPoseVelocity.setDefault(0.5);
        pidToPoseAcceleration.setDefault(0.5);
    }
}
