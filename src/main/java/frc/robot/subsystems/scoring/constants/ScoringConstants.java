package frc.robot.subsystems.scoring.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;

public class ScoringConstants {
    public static class ElevatorConstants {
        public static final Distance MAX_EXTENSION = Inches.of(72);
        public static final Distance STARTING_HEIGHT = Inches.of(30);

        public static final double UP_POSITION = 0;
        public static final double DOWN_POSITION = 1;
        public static final double POSITION_DIFF = UP_POSITION - DOWN_POSITION;

        public static final double ELEVATOR_TOLERANCE = 0.1;
    }

    public static class HopperConstants {
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(-155);
        public static final Rotation2d EXTENDED_ROTATION = Rotation2d.fromDegrees(205);

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
