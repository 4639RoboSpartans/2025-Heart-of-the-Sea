package frc.robot.subsystems.scoring.constants;

import frc.lib.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Elevator kP");
    public static TunableNumber elevatorKi = new TunableNumber("Elevator kI");
    public static TunableNumber elevatorKd = new TunableNumber("Elevator kD");
    public static TunableNumber elevatorVelocity = new TunableNumber("Elevator Velocity");
    public static TunableNumber elevatorAcceleration = new TunableNumber("Elevator Acceleration");

    static {
        elevatorKp.setDefault(10);
        elevatorKi.setDefault(0.0);
        elevatorKd.setDefault(0.0);
        elevatorVelocity.setDefault(30);
        elevatorAcceleration.setDefault(20);
    }
}
