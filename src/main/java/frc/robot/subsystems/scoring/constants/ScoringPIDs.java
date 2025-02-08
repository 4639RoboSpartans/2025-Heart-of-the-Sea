package frc.robot.subsystems.scoring.constants;

import frc.lib.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Elevator kP").withDefaultValue(25.0);
    public static TunableNumber elevatorKi = new TunableNumber("Elevator kI").withDefaultValue(0.0);
    public static TunableNumber elevatorKd = new TunableNumber("Elevator kD").withDefaultValue(0.0);
    public static TunableNumber elevatorVelocity = new TunableNumber("Elevator Velocity").withDefaultValue(20.0);
    public static TunableNumber elevatorAcceleration = new TunableNumber("Elevator Acceleration").withDefaultValue(30.0);
    public static TunableNumber elevatorKs = new TunableNumber("Elevator Ks").withDefaultValue(0.0);
    public static TunableNumber elevatorKg = new TunableNumber("Elevator Kg").withDefaultValue(0.0);
    public static TunableNumber elevatorKv = new TunableNumber("Elevator Kv").withDefaultValue(0.0);
    public static TunableNumber elevatorKa = new TunableNumber("Elevator Ka").withDefaultValue(0.0);

    public static TunableNumber wristKp = new TunableNumber("Wrist kP").withDefaultValue(100.0);
    public static TunableNumber wristKi = new TunableNumber("Wrist kI").withDefaultValue(0.0);
    public static TunableNumber wristKd = new TunableNumber("Wrist kD").withDefaultValue(2.0);
    public static TunableNumber wristVelocity = new TunableNumber("Wrist Velocity").withDefaultValue(30.0);
    public static TunableNumber wristAcceleration = new TunableNumber("Wrist Acceleration").withDefaultValue(20.0);
    public static TunableNumber wristKs = new TunableNumber("Wrist Ks").withDefaultValue(0.0);
    public static TunableNumber wristKg = new TunableNumber("Wrist Kg").withDefaultValue(0.0);
    public static TunableNumber wristKv = new TunableNumber("Wrist Kv").withDefaultValue(0.0);
    public static TunableNumber wristKa = new TunableNumber("Wrist Ka").withDefaultValue(0.0);
}
