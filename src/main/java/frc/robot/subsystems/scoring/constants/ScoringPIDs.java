package frc.robot.subsystems.scoring.constants;

import frc.lib.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Elevator kP").withDefaultValue(3.596);
    public static TunableNumber elevatorKi = new TunableNumber("Elevator kI").withDefaultValue(0.0);
    public static TunableNumber elevatorKd = new TunableNumber("Elevator kD").withDefaultValue(0.0);
    public static TunableNumber elevatorVelocity = new TunableNumber("Elevator Velocity").withDefaultValue(280.0);
    public static TunableNumber elevatorAcceleration = new TunableNumber("Elevator Acceleration").withDefaultValue(200.0);
    public static TunableNumber elevatorKs = new TunableNumber("Elevator Ks").withDefaultValue(0.34316);
    public static TunableNumber elevatorKg = new TunableNumber("Elevator Kg").withDefaultValue(0.16563);
    public static TunableNumber elevatorKv = new TunableNumber("Elevator Kv").withDefaultValue(0.11568);
    public static TunableNumber elevatorKa = new TunableNumber("Elevator Ka").withDefaultValue(0.0);

    public static TunableNumber wristKp = new TunableNumber("Wrist kP").withDefaultValue(10);
    public static TunableNumber wristKi = new TunableNumber("Wrist kI").withDefaultValue(0.0);
    public static TunableNumber wristKd = new TunableNumber("Wrist kD").withDefaultValue(0);
    public static TunableNumber wristVelocity = new TunableNumber("Wrist Velocity").withDefaultValue(5);
    public static TunableNumber wristAcceleration = new TunableNumber("Wrist Acceleration").withDefaultValue(5);
}
