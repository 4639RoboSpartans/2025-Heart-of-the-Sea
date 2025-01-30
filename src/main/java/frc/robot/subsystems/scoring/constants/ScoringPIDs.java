package frc.robot.subsystems.scoring.constants;

import frc.lib.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Elevator kP");
    public static TunableNumber elevatorKi = new TunableNumber("Elevator kI");
    public static TunableNumber elevatorKd = new TunableNumber("Elevator kD");
    public static TunableNumber elevatorVelocity = new TunableNumber("Elevator Velocity");
    public static TunableNumber elevatorAcceleration = new TunableNumber("Elevator Acceleration");

    public static TunableNumber wristKp = new TunableNumber("Wrist kP");
    public static TunableNumber wristKi = new TunableNumber("Wrist kI");
    public static TunableNumber wristKd = new TunableNumber("Wrist kD");
    public static TunableNumber wristVelocity = new TunableNumber("Wrist Velocity");
    public static TunableNumber wristAcceleration = new TunableNumber("Wrist Acceleration");

    static {
        elevatorKp.setDefault(150.0);
        elevatorKi.setDefault(0.0);
        elevatorKd.setDefault(7.0);
        elevatorVelocity.setDefault(25);
        elevatorAcceleration.setDefault(10);

        wristKp.setDefault(10.0);
        wristKi.setDefault(0.0);
        wristKd.setDefault(0.0);
        wristVelocity.setDefault(30);
        wristAcceleration.setDefault(20);
    }
}
