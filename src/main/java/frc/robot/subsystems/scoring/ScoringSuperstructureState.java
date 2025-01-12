package frc.robot.subsystems.scoring;

public enum ScoringSuperstructureState {
    HP_LOADING,
    HANDOFF,
    L1,
    L2,
    L3,
    L4;

    public final double elevatorPosition;
    public final double wristPosition;
    public final double intakeSpeed;
    public final boolean intakeUntilSeen;

    ScoringSuperstructureState(
            double elevatorPosition,
            double wristPosition,
            double intakeSpeed,
            boolean intakeUntilSeen) {
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.intakeSpeed = intakeSpeed;
        this.intakeUntilSeen = intakeUntilSeen;
    }
}
