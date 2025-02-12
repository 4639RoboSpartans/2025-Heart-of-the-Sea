package frc.robot.subsystems.scoring.constants;

import frc.lib.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Scoring PIDs/Elevator kP").withDefaultValue(3.596);
    public static TunableNumber elevatorKi = new TunableNumber("Scoring PIDs/Elevator kI").withDefaultValue(0.0);
    public static TunableNumber elevatorKd = new TunableNumber("Scoring PIDs/Elevator kD").withDefaultValue(0.0);
    public static TunableNumber elevatorVelocity = new TunableNumber("Scoring PIDs/Elevator Velocity").withDefaultValue(280.0);
    public static TunableNumber elevatorAcceleration = new TunableNumber("Scoring PIDs/Elevator Acceleration").withDefaultValue(240.0);
    public static TunableNumber elevatorKs = new TunableNumber("Scoring PIDs/Elevator Ks").withDefaultValue(0.34316);
    public static TunableNumber elevatorKg = new TunableNumber("Scoring PIDs/Elevator Kg").withDefaultValue(0.16563);
    public static TunableNumber elevatorKv = new TunableNumber("Scoring PIDs/Elevator Kv").withDefaultValue(0.11568);
    public static TunableNumber elevatorKa = new TunableNumber("Scoring PIDs/Elevator Ka").withDefaultValue(0.0);

    public static TunableNumber wristKp = new TunableNumber("Scoring PIDs/Wrist kP").withDefaultValue(10);
    public static TunableNumber wristKi = new TunableNumber("Scoring PIDs/Wrist kI").withDefaultValue(0.0);
    public static TunableNumber wristKd = new TunableNumber("Scoring PIDs/Wrist kD").withDefaultValue(0);
    public static TunableNumber wristVelocity = new TunableNumber("Scoring PIDs/Wrist Velocity").withDefaultValue(20);
    public static TunableNumber wristAcceleration = new TunableNumber("Scoring PIDs/Wrist Acceleration").withDefaultValue(40);
}
