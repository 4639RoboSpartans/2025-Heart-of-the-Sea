package frc.robot.subsystems.coralIntake.constants;

public class CoralIntakeConstants {
    public static class PivotConstants {
        public static final double downPosition = 0;
        public static final double upPosition = 1;
        public static final double positionDiff = upPosition - downPosition;

        public static final double positionTolerance = 0.01;
    }

    public static class IDs {
        public static final int pivotID = 23;
        public static final int intakeID = 1;
    }
}
