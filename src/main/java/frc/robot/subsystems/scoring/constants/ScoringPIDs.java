package frc.robot.subsystems.scoring.constants;

import frc.lib.tunable.TunableNumber;

public class ScoringPIDs {
    public static TunableNumber elevatorKp = new TunableNumber("Scoring PIDs/Elevator kP").withDefaultValue(3.796);
    public static TunableNumber elevatorKi = new TunableNumber("Scoring PIDs/Elevator kI").withDefaultValue(0.0);
    public static TunableNumber elevatorKd = new TunableNumber("Scoring PIDs/Elevator kD").withDefaultValue(0.0);
    public static TunableNumber elevatorVelocity = new TunableNumber("Scoring PIDs/Elevator Velocity").withDefaultValue(280.0);
    public static TunableNumber elevatorAcceleration = new TunableNumber("Scoring PIDs/Elevator Acceleration").withDefaultValue(120.0);
    public static TunableNumber elevatorKs = new TunableNumber("Scoring PIDs/Elevator Ks").withDefaultValue(0.34316);
    public static TunableNumber elevatorKg = new TunableNumber("Scoring PIDs/Elevator Kg").withDefaultValue(0.16563);
    public static TunableNumber elevatorKv = new TunableNumber("Scoring PIDs/Elevator Kv").withDefaultValue(0.11568);
    public static TunableNumber elevatorKa = new TunableNumber("Scoring PIDs/Elevator Ka").withDefaultValue(0.0);

    public static TunableNumber wristKp = new TunableNumber("Scoring PIDs/Wrist kP").withDefaultValue(11.0);
    public static TunableNumber wristKi = new TunableNumber("Scoring PIDs/Wrist kI").withDefaultValue(0.0);
    public static TunableNumber wristKd = new TunableNumber("Scoring PIDs/Wrist kD").withDefaultValue(0);
    public static TunableNumber wristVelocity = new TunableNumber("Scoring PIDs/Wrist Velocity").withDefaultValue(40);
    public static TunableNumber wristAcceleration = new TunableNumber("Scoring PIDs/Wrist Acceleration").withDefaultValue(80);

    public static TunableNumber simElevatorKp = new TunableNumber("Sim/Elevator kP").withDefaultValue(1);
    public static TunableNumber simElevatorKi = new TunableNumber("Sim/Elevator kI").withDefaultValue(0.0);
    public static TunableNumber simElevatorKd = new TunableNumber("Sim/Elevator kD").withDefaultValue(0.0);
    public static TunableNumber simElevatorVelocity = new TunableNumber("Sim/Elevator Velocity").withDefaultValue(300.0);
    public static TunableNumber simElevatorAcceleration = new TunableNumber("Sim/Elevator Acceleration").withDefaultValue(100.0);
    public static TunableNumber simElevatorKs = new TunableNumber("Sim/Elevator Ks").withDefaultValue(0.0);
    public static TunableNumber simElevatorKg = new TunableNumber("Sim/Elevator Kg").withDefaultValue(0.63195475);
    public static TunableNumber simElevatorKv = new TunableNumber("Sim/Elevator Kv").withDefaultValue(0.085);
    public static TunableNumber simElevatorKa = new TunableNumber("Sim/Elevator Ka").withDefaultValue(0.0);
}
