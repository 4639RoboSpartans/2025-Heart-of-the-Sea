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

        public static final Trigger rotationResetTrigger = driverController.A_BUTTON.and(driverController.B_BUTTON);

        /*public static final Trigger reefLeftPoses = driverController.RIGHT_BUMPER;
        public static final Trigger reefRightPoses = driverController.LEFT_BUMPER;

        public static final Trigger reefRight = driverController.LEFT_TRIGGER;
        public static final Trigger reefLeft = driverController.RIGHT_TRIGGER;

        public static final Trigger PathfindReef_0 = reefLeftPoses.and(reefLeft.or(reefRight)).and(driverController.Y_BUTTON);
        public static final Trigger PathfindReef_1 = reefLeftPoses.and(reefLeft.or(reefRight)).and(driverController.B_BUTTON);
        public static final Trigger PathfindReef_2 = reefLeftPoses.and(reefLeft.or(reefRight)).and(driverController.A_BUTTON);
        public static final Trigger PathfindReef_3 = reefRightPoses.and(reefLeft.or(reefRight)).and(driverController.Y_BUTTON);
        public static final Trigger PathfindReef_4 = reefRightPoses.and(reefLeft.or(reefRight)).and(driverController.X_BUTTON);
        public static final Trigger PathfindReef_5 = reefRightPoses.and(reefLeft.or(reefRight)).and(driverController.A_BUTTON);*/

        //these need to be changed over to driver controller
        public static final Trigger L2AlgaeTrigger = driverController.POV_DOWN;
        public static final Trigger L3AlgaeTrigger = driverController.POV_UP;
        public static final Trigger ProcessorTrigger = driverController.Y_BUTTON;
        public static final Trigger BargeScoringTrigger = driverController.X_BUTTON;

        public static final Trigger alignReefRight = driverController.RIGHT_BUMPER;
        public static final Trigger alignReefLeft = driverController.LEFT_BUMPER;

        public static final Trigger reefAlign = driverController.RIGHT_TRIGGER;
        public static final Trigger coralStationAlign = driverController.B_BUTTON;
        public static final Trigger bargeScoringAlign = driverController.XBOX_START_BUTTON;
        public static final Trigger processorAlign = driverController.XBOX_BACK_BUTTON;
    }

    public static class Operator {
        //Setpoints +Operator Controls
        public static final Trigger L1Trigger = operatorController.LEFT_TRIGGER;
        public static final Trigger L2Trigger = operatorController.LEFT_BUMPER;
        public static final Trigger L3Trigger = operatorController.RIGHT_BUMPER;
        public static final Trigger L4Trigger = operatorController.RIGHT_TRIGGER;

        public static final Trigger HPLoadingTrigger = operatorController.X_BUTTON;

        public static final Trigger ScoringIdleTrigger = operatorController.Y_BUTTON;

        // public static final Trigger FunnelTrigger = operatorController.RIGHT_STICK;

        // Micro adjustment controls
        public static final DoubleSupplier MicroElevatorAdjustment = () -> operatorController.POV_UP() - operatorController.POV_DOWN();
        public static final DoubleSupplier MicroWristAdjustment = () -> operatorController.POV_RIGHT() - operatorController.POV_LEFT();

        //Manual override controls
        public static Trigger ToggleManualControlTrigger = operatorController.LEFT_STICK;
        public static Trigger HomingCommandTrigger = operatorController.RIGHT_STICK;
        public static Trigger homingWristCommandTrigger = operatorController.A_BUTTON.and(operatorController.B_BUTTON);

        public static final DoubleSupplier ManualControlWrist = () -> operatorController.rightStickY() * 0.5 + 0.5;
        public static final DoubleSupplier ManualControlElevator = operatorController::leftStickY;
        public static final DoubleSupplier ManualControlIntake = () -> (operatorController.A_BUTTON() - operatorController.B_BUTTON()) * 0.75;

        public static final DoubleSupplier ManualControlFunnel = () -> operatorController.rightStickX();
    }
}
