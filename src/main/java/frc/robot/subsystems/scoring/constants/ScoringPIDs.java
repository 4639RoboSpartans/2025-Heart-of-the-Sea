package frc.robot.subsystems.scoring.constants;

import frc.lib.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Elevator kP");
    public static TunableNumber elevatorKi = new TunableNumber("Elevator kI");
    public static TunableNumber elevatorKd = new TunableNumber("Elevator kD");
    public static TunableNumber elevatorVelocity = new TunableNumber("Elevator Velocity");
    public static TunableNumber elevatorAcceleration = new TunableNumber("Elevator Acceleration");
    public static TunableNumber elevatorKs = new TunableNumber("Elevator Ks");
    public static TunableNumber elevatorKg = new TunableNumber("Elevator Kg");
    public static TunableNumber elevatorKv = new TunableNumber("Elevator Kv");
    public static TunableNumber elevatorKa = new TunableNumber("Elevator Ka");

    public static TunableNumber wristKp = new TunableNumber("Wrist kP");
    public static TunableNumber wristKi = new TunableNumber("Wrist kI");
    public static TunableNumber wristKd = new TunableNumber("Wrist kD");
    public static TunableNumber wristVelocity = new TunableNumber("Wrist Velocity");
    public static TunableNumber wristAcceleration = new TunableNumber("Wrist Acceleration");
    public static TunableNumber wristKs = new TunableNumber("Wrist Ks");
    public static TunableNumber wristKg = new TunableNumber("Wrist Kg");
    public static TunableNumber wristKv = new TunableNumber("Wrist Kv");
    public static TunableNumber wristKa = new TunableNumber("Wrist Ka");

    static {
        elevatorKp.setDefault(0);
        elevatorKi.setDefault(0);
        elevatorKd.setDefault(0);
        elevatorVelocity.setDefault(0.3);
        elevatorAcceleration.setDefault(0.3);
        elevatorKs.setDefault(0);
        elevatorKg.setDefault(0.625);
        elevatorKv.setDefault(0);
        elevatorKa.setDefault(0);

        wristKp.setDefault(100.0);
        wristKi.setDefault(0.0);
        wristKd.setDefault(2.0);
        wristVelocity.setDefault(30);
        wristAcceleration.setDefault(20);
        wristKs.setDefault(0.0);
        wristKg.setDefault(0.0);
        wristKv.setDefault(0.0);
        wristKa.setDefault(0.0);
    }
}
