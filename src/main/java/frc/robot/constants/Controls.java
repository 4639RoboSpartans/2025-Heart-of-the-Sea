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
}
