package frc.robot.constants;

import frc.lib.oi.OI;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
/*
 * Driver Controls:
 * - Left Stick Y-> 
 * - Left Stick X-> 
 * - Right Stick Y-> 
 * - Right Stick X->
 * - D Pad Left ->
 * - D Pad Right ->
 * - D Pad Up 
 * - D Pad Down
 * - X Button
 * - Y Button
 * - A Button
 * - B Button
 * - RB Button
 * - LB Button
 * - R Trigger
 * - L Trigger
 * 
 * Operator Controls:
 * 
 */

public class Controls {
    public static class Driver {
        public static final DoubleSupplier SwerveForwardAxis = OI.getInstance().driverController()::leftStickY;
        public static final DoubleSupplier SwerveStrafeAxis = OI.getInstance().driverController()::leftStickX;
        public static final DoubleSupplier SwerveRotationAxis = OI.getInstance().driverController()::rightStickX;

        public static final Trigger precisionTrigger = OI.getInstance().driverController().LEFT_TRIGGER;

        public static final Trigger rotationResetTrigger = OI.getInstance().driverController().A_BUTTON
                                                            .and(OI.getInstance().driverController().B_BUTTON);
    }
    public static class Operator{
        //manual override controls
        public static final DoubleSupplier manualElevator = OI.getInstance().operatorController()::leftStickY;
        public static final DoubleSupplier manualWrist = OI.getInstance().operatorController()::rightStickY;
        public static final DoubleSupplier manualScoringOuIntake = OI.getInstance().operatorController()::rightStickX;
    }
}
