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

<<<<<<< HEAD
    public static TunableNumber wristKp = new TunableNumber("Wrist kP").withDefaultValue(100.0);
    public static TunableNumber wristKi = new TunableNumber("Wrist kI").withDefaultValue(0.0);
    public static TunableNumber wristKd = new TunableNumber("Wrist kD").withDefaultValue(2.0);
    public static TunableNumber wristVelocity = new TunableNumber("Wrist Velocity").withDefaultValue(30.0);
    public static TunableNumber wristAcceleration = new TunableNumber("Wrist Acceleration").withDefaultValue(20.0);
    public static TunableNumber wristKs = new TunableNumber("Wrist Ks").withDefaultValue(0.0);
    public static TunableNumber wristKg = new TunableNumber("Wrist Kg").withDefaultValue(0.0);
    public static TunableNumber wristKv = new TunableNumber("Wrist Kv").withDefaultValue(0.0);
    public static TunableNumber wristKa = new TunableNumber("Wrist Ka").withDefaultValue(0.0);
=======
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
>>>>>>> 8bc0ed923b6486f611454f74519cb33e1e3fbc3c
}
