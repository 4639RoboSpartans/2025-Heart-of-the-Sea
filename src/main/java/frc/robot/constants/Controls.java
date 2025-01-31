package frc.robot.constants;

import frc.lib.oi.OI;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controls {
    public static class Driver {
        public static final DoubleSupplier SwerveForwardAxis = OI.getInstance().driverController()::leftStickY;
        public static final DoubleSupplier SwerveStrafeAxis = OI.getInstance().driverController()::leftStickX;
        public static final DoubleSupplier SwerveRotationAxis = OI.getInstance().driverController()::rightStickX;

        public static final Trigger precisionTrigger = OI.getInstance().driverController().LEFT_TRIGGER;

        public static final Trigger rotationResetTrigger = OI.getInstance().driverController().A_BUTTON
                                                            .and(OI.getInstance().driverController().B_BUTTON);
    }

    public static class Operator {
        public static final Trigger L1Trigger = OI.getInstance().operatorController().LEFT_TRIGGER;
        public static final Trigger L2Trigger = OI.getInstance().operatorController().RIGHT_TRIGGER;
        public static final Trigger L3Trigger = OI.getInstance().operatorController().LEFT_BUMPER;
        public static final Trigger L4Trigger = OI.getInstance().operatorController().RIGHT_BUMPER;

        public static final Trigger L2AlgaeTrigger = OI.getInstance().operatorController().X_BUTTON;
        public static final Trigger L3AlgaeTrigger = OI.getInstance().operatorController().Y_BUTTON;

        public static final Trigger BargeScoringTrigger = OI.getInstance().operatorController().A_BUTTON;

        public static final Trigger HPLoadingTrigger = OI.getInstance().operatorController().B_BUTTON;
    }
}
