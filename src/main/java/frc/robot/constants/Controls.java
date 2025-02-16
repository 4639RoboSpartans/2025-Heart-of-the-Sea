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

        public static final Trigger reefLeftPoses = driverController.RIGHT_BUMPER;
        public static final Trigger reefRightPoses = driverController.LEFT_BUMPER;

        public static final Trigger reefRight = driverController.LEFT_TRIGGER;
        public static final Trigger reefLeft = driverController.RIGHT_TRIGGER;

        public static final Trigger PathfindReef_0 = reefLeftPoses.and(reefLeft.or(reefRight))
            .and(driverController.Y_BUTTON);
        public static final Trigger PathfindReef_1 = reefLeftPoses.and(reefLeft.or(reefRight))
            .and(driverController.B_BUTTON);
        public static final Trigger PathfindReef_2 = reefLeftPoses.and(reefLeft.or(reefRight))
            .and(driverController.A_BUTTON);
        public static final Trigger PathfindReef_3 = reefRightPoses.and(reefLeft.or(reefRight))
            .and(driverController.Y_BUTTON);
        public static final Trigger PathfindReef_4 = reefRightPoses.and(reefLeft.or(reefRight))
            .and(driverController.X_BUTTON);
        public static final Trigger PathfindReef_5 = reefRightPoses.and(reefLeft.or(reefRight))
            .and(driverController.A_BUTTON);
    }

    public static class Operator {
        public static final Trigger L1Trigger = operatorController.LEFT_TRIGGER;
        public static final Trigger L2Trigger = operatorController.RIGHT_TRIGGER;
        public static final Trigger L3Trigger = operatorController.LEFT_BUMPER;
        public static final Trigger L4Trigger = operatorController.RIGHT_BUMPER;

        public static final Trigger L2AlgaeTrigger = operatorController.X_BUTTON;
        public static final Trigger L3AlgaeTrigger = operatorController.Y_BUTTON;

        public static final Trigger BargeScoringTrigger = operatorController.A_BUTTON;

        public static final Trigger HPLoadingTrigger = operatorController.B_BUTTON;

        public static final Trigger HoldTrigger = operatorController.POV_UP;

        public static Trigger ToggleManualControlTrigger = operatorController.A_BUTTON.and(operatorController.B_BUTTON).and(operatorController.POV_UP);

        public static final DoubleSupplier ManualControlHopper = () -> (
            (operatorController.POV_RIGHT.getAsBoolean() ? 1 : 0) -
                (operatorController.POV_LEFT.getAsBoolean() ? 1 : 0)
        );
        public static final DoubleSupplier ManualControlElevator = () -> (
            (operatorController.POV_UP.getAsBoolean() ? 1 : 0) -
                (operatorController.POV_DOWN.getAsBoolean() ? 1 : 0)
        );
    }
}
