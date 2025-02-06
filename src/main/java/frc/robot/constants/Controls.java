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
 * - Left Stick Y-> Manual Elevator Up/Down
 * - Left Stick X-> 
 * - Right Stick Y-> Manual Wrist
 * - Right Stick X-> Intake/Outtake Scoring
 * - D Pad Left ->
 * - D Pad Right ->
 * - D Pad Up ->
 * - D Pad Down -> Stow Elevator
 * - X Button -> L2 Scoring
 * - Y Button -> L1 Scoring
 * - A Button -> Processor Score
 * - B Button -> Barge Score
 * - RB Button -> L3 Scoring
 * - LB Button
 * - R Trigger -> L4 Scoring
 * - L Trigger
 */

public class Controls {
    public static class Driver {
        //swerve drive
        public static final DoubleSupplier SwerveForwardAxis = OI.getInstance().driverController()::leftStickY;
        public static final DoubleSupplier SwerveStrafeAxis = OI.getInstance().driverController()::leftStickX;
        public static final DoubleSupplier SwerveRotationAxis = OI.getInstance().driverController()::rightStickX;

        public static final Trigger precisionTrigger = OI.getInstance().driverController().LEFT_TRIGGER;
        // press A and B at the same to reset swerve rotation to the current heading
        public static final Trigger rotationResetTrigger = OI.getInstance().driverController().A_BUTTON.and(OI.getInstance().driverController().B_BUTTON); 
    }
    public static class Operator{
        //manual override controls
        public static final DoubleSupplier manualElevator = OI.getInstance().operatorController()::leftStickY;
        public static final DoubleSupplier manualWrist = OI.getInstance().operatorController()::rightStickY;
        public static final DoubleSupplier manualScoringOuIntake = OI.getInstance().operatorController()::rightStickX;
        //manual operator resets
        public static final Trigger elevatorResetSetPointTrigger = OI.getInstance().operatorController().A_BUTTON.and(OI.getInstance().driverController().B_BUTTON);
        //normal set points and controls
        public static final Trigger L4Trigger = OI.getInstance().operatorController().RIGHT_TRIGGER;
        public static final Trigger L3Trigger = OI.getInstance().operatorController().RIGHT_BUMPER;
        public static final Trigger L2Trigger = OI.getInstance().operatorController().X_BUTTON;
        public static final Trigger L1Trigger = OI.getInstance().operatorController().Y_BUTTON;
        public static final Trigger processorScoreTrigger = OI.getInstance().operatorController().A_BUTTON;
        public static final Trigger bargeScoreTrigger = OI.getInstance().operatorController().B_BUTTON;
        public static final Trigger stowElevatorTrigger = OI.getInstance().operatorController().POV_DOWN;
    }
}
