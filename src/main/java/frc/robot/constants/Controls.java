package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.Controller;
import frc.lib.oi.OI;

import java.util.function.DoubleSupplier;
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
 */

public class Controls {
    private static final Controller driverController = OI.getInstance().driverController();
    private static final Controller operatorController = OI.getInstance().operatorController();

    public static class Driver {
        public static final DoubleSupplier SwerveForwardAxis = driverController::leftStickY;
        public static final DoubleSupplier SwerveStrafeAxis = driverController::leftStickX;
        public static final DoubleSupplier SwerveRotationAxis = driverController::rightStickX;

        public static final Trigger precisionTrigger = driverController.LEFT_TRIGGER;

        public static final Trigger rotationResetTrigger = driverController.A_BUTTON
            .and(driverController.B_BUTTON);

        public static final Trigger targetRight = driverController.RIGHT_BUMPER;
        public static final Trigger targetLeft = driverController.LEFT_BUMPER;

        public static final Trigger reefAlign = driverController.POV_UP;
        public static final Trigger coralStationAlign = driverController.POV_DOWN;

        public static final Trigger L2AlgaeTrigger = driverController.X_BUTTON;
        public static final Trigger L3AlgaeTrigger = driverController.Y_BUTTON;
        public static final Trigger ProcessorTrigger = driverController.POV_RIGHT;
        public static final Trigger BargeScoringTrigger = driverController.A_BUTTON;
    }

    public static class Operator {
        //Setpoints +Operator Controls
        public static final Trigger L1Trigger = operatorController.LEFT_TRIGGER;
        public static final Trigger L2Trigger = operatorController.LEFT_BUMPER;
        public static final Trigger L3Trigger = operatorController.RIGHT_BUMPER;
        public static final Trigger L4Trigger = operatorController.RIGHT_TRIGGER;
        //till need to add 

        public static final Trigger HPLoadingTrigger = operatorController.B_BUTTON;

        public static final Trigger HoldTrigger = operatorController.POV_UP;

        //Manual Override Controls
        public static Trigger ToggleManualControlTrigger = operatorController.LEFT_STICK;

        public static final DoubleSupplier ManualControlHopper = () -> (
            (operatorController.POV_RIGHT.getAsBoolean() ? 1 : 0) -
                (operatorController.POV_LEFT.getAsBoolean() ? 1 : 0)
        );
        public static final DoubleSupplier ManualControlElevator = () -> (
            (operatorController.POV_UP.getAsBoolean() ? 1 : 0) -
                (operatorController.POV_DOWN.getAsBoolean() ? 1 : 0)
        );
        public static final DoubleSupplier ManualControlIntake = () -> (
            (operatorController.A_BUTTON.getAsBoolean() ? 1 : 0) -
                (operatorController.B_BUTTON.getAsBoolean() ? 1 : 0)
        );
    }
}
