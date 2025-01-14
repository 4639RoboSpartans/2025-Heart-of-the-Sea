package frc.robot.subsystems.scoring.constants;

public class ScoringConstants {
    public static class ElevatorConstants {
        public static final double UP_POSITION = 0;
        public static final double DOWN_POSITION = 1;
        public static final double POSITION_DIFF = UP_POSITION - DOWN_POSITION;

        public static final double ELEVATOR_TOLERANCE = 0.1;
    }

    public static class WristConstants {
        public static final double UP_POSITION = 0;
        public static final double DOWN_POSITION = 1;
        public static final double POSITION_DIFF = UP_POSITION - DOWN_POSITION;

        public static final double WRIST_ABSOLUTE_DOWN_POSITION = 0;

        public static final double WRIST_TOLERANCE = 0.1;
    }

    public static class IDs {
        public static final int ElevatorLeftID = 21;
        public static final int ElevatorRightID = 22;

        public static final int IntakeMotorID = 10;
        public static final int WristMotorID = 11;

        public static final int WristEncoderID = 12;

        public static final int LaserCANID = 13;
    }
}
