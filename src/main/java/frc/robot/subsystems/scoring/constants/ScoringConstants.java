package frc.robot.subsystems.scoring.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.UnitConvertor;

import static edu.wpi.first.units.Units.Inches;

public final class ScoringConstants {
    public static final class ElevatorConstants {
        // The range of the elevator
        public static final Distance MAX_EXTENSION = Inches.of(84);
        // The initial height of the elevator
        public static final Distance STARTING_HEIGHT = Inches.of(12);

        public static double UP_POSITION = 64;  // These
        public static double DOWN_POSITION = 2; // are correct for sure

        public static final double ELEVATOR_TOLERANCE = 1;

        public static final UnitConvertor<Double, Double> ProportionToPosition = UnitConvertor.linearConvertingRange(
            0, 1, DOWN_POSITION, UP_POSITION
        );

        public static final UnitConvertor<Double, Distance> ProportionToHeight = UnitConvertor.linear(
            MAX_EXTENSION.in(Inches), STARTING_HEIGHT.in(Inches), false
        ).then(UnitConvertor.toDistance(Inches));

        public static final UnitConvertor<Double, Distance> PositionToHeight = UnitConvertor.compose(
            ProportionToPosition.inverted(),
            ProportionToHeight
        );

        public static final class ElevatorSetpoints {
            //Elevator proportions
            public static final double IDLE_Proportion = 0.02; // TODO: figure out how to fix
            public static final double HP_Proportion = 0.02;
            public static final double Processor_Proportion = ProportionToPosition.convertBackwards(2.620); // TODO: tune
            public static final double L1_Proportion = 0.15; // TODO: tune
            // TODO: above


            public static final double L2_Proportion = 0.26;
            public static final double L3_Proportion = 0.47;
            public static final double L4_Proportion = 0.85;

            // TODO: below
            public static final double L2_ALGAE_Proportion = 0.32; // MAYBE
            public static final double L3_ALGAE_Proportion = ProportionToPosition.convertBackwards(32.0);
            public static final double Barge_Proportion = ProportionToPosition.convertBackwards(62.3);

            public static final double Ground_Intake_Proportion = 0.0;
            public static final double Homing_Proportion = -0.04;
        }
    }

    public static class EndEffectorConstants {
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(-200);
        public static final Rotation2d IDLE_ROTATION = Rotation2d.fromDegrees(30);

        public static boolean usingRelativeEncoder = false;
        public static final double IDLE_POSITION = 0;
        public static final double EXTENDED_POSITION = usingRelativeEncoder ? -31.846 : 0.95;

        public static Rotation2d RotationStartingPosition = Rotation2d.kZero;

        public static final UnitConvertor<Double, Double> relativeEncoderMeasurementToAbsoluteMeasurement = UnitConvertor.linearConvertingRange(0, 0.89, 0, -31.846);

        public static final UnitConvertor<Double, Double> RotationFractionToMotorPosition = UnitConvertor.linearConvertingRange(
            0, 1, IDLE_POSITION, EXTENDED_POSITION
        );
        public static final UnitConvertor<Double, Rotation2d> ProportionToRotation = UnitConvertor.linear(
            MAX_ROTATION.getRadians(), IDLE_ROTATION.getRadians(), false
        ).then(UnitConvertor.radiansToRotation2d());
        public static final UnitConvertor<Double, Rotation2d> PositionToRotation = UnitConvertor.compose(
            RotationFractionToMotorPosition.inverted(),
            ProportionToRotation
        );

        public static final double WRIST_TOLERANCE = 0.03;

        //Limits for Intake
        public static final double IntakeForwardSoftLimit = 40;
        public static final double IntakeReverseSoftLimit = 40;
        public static final int IntakeCurrentLimit = 50;

        //Limits for Wrist
        public static final double WristForwardSoftLimit = 30;
        public static final double WristReverseSoftLimit = 30;
        public static final int WristCurrentLimit = 30;

        public static final class IntakeSpeeds {
            public static final double Intake_HP_Speed = 0.5;
            public static final double Intake_Processor_Speed = 0.5;
            public static final double Intake_L1_Speed = 0.2;
            public static final double Intake_L2_Speed = -0.5;
            public static final double Intake_L3_Speed = -0.5;
            public static final double Intake_L4_Speed = -0.5;
            public static final double Intake_L2_ALGAE_Speed = 1.0;
            public static final double Intake_L3_ALGAE_Speed = 1.0;
            public static final double Intake_Barge_Speed = 0.4;
            public static final double Intake_Idle_Speed = 0.5;
        }

        public static final class WristSetpoints {
            //Wrist Proportions
            public static final double Wrist_IDLE_Proportion = 0.2; // For now, because shaft collision
            public static final double Wrist_ALGAESTOW_Proportion = 0.5;
            public static final double Wrist_HP_Proportion = 0.2;
            public static final double Wrist_Processor_Proportion = 0.709;
            public static final double Wrist_L1_Proportion = 1.0;
            // TODO: above

            public static final double Wrist_L2_Proportion = 0.95;
            public static final double Wrist_L3_Proportion = 0.95;
            public static final double Wrist_L4_Proportion = 1.0;

            // TODO: below
            public static final double Wrist_L2_ALGAE_Proportion = 0.82;
            public static final double Wrist_L3_ALGAE_Proportion = 0.763;
            public static final double Wrist_Barge_Proportion = 0.534;
            public static final double Wrist_Transition_Proportion = 0.5;

            public static final Double Wrist_Ground_Intake_Proportion = 0.845;
        }

        public static final Translation3d Hopper3DSimOffset = new Translation3d(
            0.275,
            0,
            0.38125
        );
    }

    public static class IDs {
        public static final int ElevatorLeftID = 21;
        public static final int ElevatorRightID = 22;
        public static final String ElevatorCANBusName = "MainCANivore";

        public static final int IntakeMotorID = 23;

        public static final int WristEncoderDIOPort = 5;
        public static final int WristMotorID = 24;
        public static final String WristCANBusName = "MainCANivore";

        public static final int LaserCANID = 56;
    }
}