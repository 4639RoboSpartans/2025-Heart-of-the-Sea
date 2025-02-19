package frc.robot.subsystems.scoring.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.UnitConvertor;

import static edu.wpi.first.units.Units.Inches;

// L2 Coral = E 29.5 W -36.9
// L3 Coral = E 40.9 W -36.9
// L4 Coral = E 67.7 W -40.5 (max proportion)
// L2 Algae = E 30.54 W -30
// L3 Algae = E 41.3 W -23

public final class ScoringConstants {
    public static final class ElevatorConstants {
        // The range of the elevator
        public static final Distance MAX_EXTENSION = Inches.of(84);
        // The initial height of the elevator
        public static final Distance STARTING_HEIGHT = Inches.of(12);

        public static double UP_POSITION = 63.77;
        public static double DOWN_POSITION = 1.2;

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

        public static final class Proportions {
            //Elevator proportions
            public static final double IDLE_Proportion = 0.0;
            public static final double HP_Proportion = 0.1;
            public static final double Processor_Proportion =0.1;
            public static final double L1_Proportion = 0.15;
            public static final double L2_Proportion = ProportionToPosition.convertBackwards(29.5);
            public static final double L3_Proportion = ProportionToPosition.convertBackwards(40.9);
            public static final double L4_Proportion = ProportionToPosition.convertBackwards(67.7);
            public static final double L2_ALGAE_Proportion = ProportionToPosition.convertBackwards(30.5);
            public static final double L3_ALGAE_Proportion = ProportionToPosition.convertBackwards(41.3);
            public static final double Barge_Proportion = 1;
            public static final double Transition_Proportion = 0.0;
        }
    }

    public static class EndEffectorConstants {
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(-200);
        public static final Rotation2d IDLE_ROTATION = Rotation2d.fromDegrees(30);

        public static final double IDLE_POSITION = 0;
        public static final double EXTENDED_POSITION = -40.5;

        public static final UnitConvertor<Double, Double> ProportionToPosition = UnitConvertor.linearConvertingRange(
            0, 1, IDLE_POSITION, EXTENDED_POSITION
        );
        public static final UnitConvertor<Double, Rotation2d> ProportionToRotation = UnitConvertor.linear(
            MAX_ROTATION.getRadians(), IDLE_ROTATION.getRadians(), false
        ).then(UnitConvertor.radiansToRotation2d());
        public static final UnitConvertor<Double, Rotation2d> PositionToRotation = UnitConvertor.compose(
            ProportionToPosition.inverted(),
            ProportionToRotation
        );

        public static final double WRIST_TOLERANCE = 0.1;

        //Limits for Intake
        public static final double IntakeForwardSoftLimit = 40;
        public static final double IntakeReverseSoftLimit = 40;
        public static final int IntakeCurrentLimit = 50;

        //Limits for Wrist
        public static final double WristForwardSoftLimit = 30;
        public static final double WristReverseSoftLimit = 30;
        public static final int WristCurrentLimit = 30;

        //Intake Speeds
        public static final double Intake_IDLE_Speed = 0.0;
        public static final double Intake_HP_Speed = 0.5;
        public static final double Intake_Processor_Speed = 0.5;
        public static final double Intake_L1_Speed = 0.5;
        public static final double Intake_L2_Speed = 0.5;
        public static final double Intake_L3_Speed = 0.5;
        public static final double Intake_L4_Speed = 0.5;
        public static final double Intake_L2_ALGAE_Speed = -0.5;
        public static final double Intake_L3_ALGAE_Speed = -0.5;
        public static final double Intake_Barge_Speed = 1.0;
        public static final double Intake_Transition_Speed = 0.0;

        public static final class Proportions {

            //Wrist Proportions
            public static final double Wrist_IDLE_Proportion = 0.0;
            public static final double Wrist_HP_Proportion = 0.0;
            public static final double Wrist_Processor_Proportion = 0.0;
            public static final double Wrist_L1_Proportion = 1.0;
            public static final double Wrist_L2_Proportion = ProportionToPosition.convertBackwards(-36.9);
            public static final double Wrist_L3_Proportion = ProportionToPosition.convertBackwards(-36.9);
            public static final double Wrist_L4_Proportion = ProportionToPosition.convertBackwards(-40.5);
            public static final double Wrist_L2_ALGAE_Proportion = ProportionToPosition.convertBackwards(-30.);
            public static final double Wrist_L3_ALGAE_Proportion = ProportionToPosition.convertBackwards(-23.);
            public static final double Wrist_Barge_Proportion = ProportionToPosition.convertBackwards(-16.3);
            public static final double Wrist_Transition_Proportion = 0.4;
        }

        public static final Translation3d Hopper3DSimOffset = new Translation3d(
            0.275,
            0,
            0.38125
        );
    }

    public static class FunnelConstants{
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(-200);
        public static final Rotation2d DOWN_ROTATION = Rotation2d.fromDegrees(30);

        public static final double DOWN_POSITION = 0;
        public static final double UP_POSITION = 10.;

        public static final UnitConvertor<Double, Double> ProportionToPosition = UnitConvertor.linearConvertingRange(
            0, 1, DOWN_POSITION, UP_POSITION
        );
        public static final UnitConvertor<Double, Rotation2d> ProportionToRotation = UnitConvertor.linear(
            MAX_ROTATION.getRadians(), DOWN_ROTATION.getRadians(), false
        ).then(UnitConvertor.radiansToRotation2d());
        public static final UnitConvertor<Double, Rotation2d> PositionToRotation = UnitConvertor.compose(
            ProportionToPosition.inverted(),
            ProportionToRotation
        );

        public static final double Pivot_Speed = .2;

        public static final double PivotForwardSoftLimit = 30;
        public static final double PivotReverseSoftLimit = 30;
        public static final int PivotCurrentLimit = 30;
    }

    public static class IDs {
        public static final int ElevatorLeftID = 21;
        public static final int ElevatorRightID = 22;

        public static final String ElevatorCANBusName = "MainCANivore";
        public static final String WristCANBusName = "MainCANivore";

        public static final int IntakeMotorID = 23;
        public static final int WristMotorID = 24;

        public static final int WristEncoderID = 25;

        public static final int FunnelPivotMotorID = 27;

        public static final int LaserCANID = 26;
    }
}
