package frc.robot.subsystems.scoring.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;

public final class ScoringConstants {
    public static final class ElevatorConstants {
        public static final Distance MAX_EXTENSION = Inches.of(84);
        public static final Distance STARTING_HEIGHT = Inches.of(12);

        public static double UP_POSITION = 63;
        public static double DOWN_POSITION = 0;
        public static final double POSITION_DIFF = UP_POSITION - DOWN_POSITION;

        public static final double ELEVATOR_TOLERANCE = 0.01;

        public static final class Proportions{

            //Elevator proportions
            public static final double IDLE_Proportion = 0.0;
            public static final double HP_Proportion = 0.5;
            public static final double L1_Proportion = 0.15;
            public static final double L2_Proportion = 0.25;
            public static final double L3_Proportion = 0.4;
            public static final double L4_Proportion = 0.8;
            public static final double L2_ALGAE_Proportion = 0.55;
            public static final double L3_ALGAE_Proportion = 0.65;
            public static final double Barge_Proportion = 1;

            public static double positionToProportion(double position) {
                return (position - DOWN_POSITION) / POSITION_DIFF;
            }
        }
    }

    public static class HopperConstants {
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(-155);
        public static final Rotation2d IDLE_ROTATION = Rotation2d.fromDegrees(30);

        public static final double IDLE_POSITION = 0;
        public static final double EXTENDED_POSITION = 1;
        public static final double POSITION_DIFF = IDLE_POSITION - EXTENDED_POSITION;

        public static final double WRIST_ABSOLUTE_DOWN_POSITION = 0;

        public static final double WRIST_TOLERANCE = 0.01;

        public static final class Proportions{

            //Wrist Proportions
            public static final double Wrist_IDLE_Proportion = 0.0;
            public static final double Wrist_HP_Proportion = 0.0;
            public static final double Wrist_L1_Proportion = 1.0;
            public static final double Wrist_L2_Proportion = 0.25;
            public static final double Wrist_L3_Proportion = 0.25;
            public static final double Wrist_L4_Proportion = 0.5;
            public static final double Wrist_L2_ALGAE_Proportion = 1.0;
            public static final double Wrist_L3_ALGAE_Proportion = 1.0;
            public static final double Wrist_Barge_Proportion = 0.5;

            public static double positionToProportion(double position) {
                return (position - IDLE_POSITION) / POSITION_DIFF;
            }
        }
    }

    public static class IDs {
        public static final int ElevatorLeftID = 21;
        public static final int ElevatorRightID = 22;

        public static final String ElevatorCANBusName = "MainCANivore";

        public static final int IntakeMotorID = 10;
        public static final int WristMotorID = 11;

        public static final int WristEncoderID = 12;

        public static final int LaserCANID = 13;
    }
}
