package frc.lib.oi;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private static final double DEADZONE_VALUE = 0.1;

    private final XboxController controller;

    public final Trigger
        A_BUTTON,
        B_BUTTON,
        X_BUTTON,
        Y_BUTTON,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        LEFT_STICK,
        RIGHT_STICK,
        POV_UP,
        POV_RIGHT,
        POV_DOWN,
        POV_LEFT,
        LEFT_TRIGGER,
        RIGHT_TRIGGER;

    public Controller(int joystickID) {
        controller = new XboxController(joystickID);

        A_BUTTON = new Trigger(controller::getAButton);
        B_BUTTON = new Trigger(controller::getBButton);
        X_BUTTON = new Trigger(controller::getXButton);
        Y_BUTTON = new Trigger(controller::getYButton);
        LEFT_BUMPER = new Trigger(controller::getLeftBumperButton);
        RIGHT_BUMPER = new Trigger(controller::getRightBumperButton);
        LEFT_STICK = new Trigger(controller::getLeftStickButton);
        RIGHT_STICK = new Trigger(controller::getRightStickButton);
        POV_UP = new Trigger(() -> controller.getPOV() == 0);
        POV_RIGHT = new Trigger(() -> controller.getPOV() == 90);
        POV_DOWN = new Trigger(() -> controller.getPOV() == 180);
        POV_LEFT = new Trigger(() -> controller.getPOV() == 270);
        LEFT_TRIGGER = new Trigger(() -> controller.getLeftTriggerAxis() > 0.5);
        RIGHT_TRIGGER = new Trigger(() -> controller.getRightTriggerAxis() > 0.5);
    }

    /**
     * Applies a dead zone. The controllers sticks are a little loose and so there is a margin of error for where they
     * should be considered neutral
     */
    private static double applyDeadzone(double value) {
        // When the value is LESS than the magic number, the
        // joystick is in the loose position so return zero - as if the
        // joystick was not moved
        if (Math.abs(value) < DEADZONE_VALUE) {
            return 0;
        }

        // When the joystick is MORE than the magic number,
        // scale the value so that the point right after the
        // deadzone is 0 so the robot does not jerk forward
        // when it passes the deadzone.
        return Math.signum(value) * ((Math.abs(value) - DEADZONE_VALUE) / (1 - DEADZONE_VALUE));
    }

    /**
     * Gets the left joystick's x axis, a number from -1 (left) to 1 (right)
     */
    public double leftStickX() {
        return applyDeadzone(controller.getLeftX());
    }

    /**
     * Gets the left joystick's y axis, a number from -1 (down) to 1 (up)
     */
    public double leftStickY() {
        return applyDeadzone(-controller.getLeftY());
    }

    /**
     * Gets the angle of the left joystick measured counterclockwise from the positive x direction. The value returned
     * by this method should not be relied on when the joystick is neutral
     */
    public Rotation2d leftAngle() {
        return Rotation2d.fromRadians(Math.atan2(leftStickY(), leftStickX()));
    }

    /**
     * Gets the distance of the right joystick from neutral. 
     */
    public double leftHypot() {
        return Math.hypot(leftStickX(), leftStickY());
    }

    /**
     * Gets the right joystick's x axis, a number from -1 (left) to 1 (right)
     */
    public double rightStickX() {
        return applyDeadzone(-controller.getRightX());
    }

    /**
     * Gets the right joystick's y axis, a number from -1 (down) to 1 (up)
     */
    public double rightStickY() {
        return applyDeadzone(-controller.getRightY());
    }

    /**
     * Gets the angle of the right joystick measured counterclockwise from the positive x direction. The value returned
     * by this method should not be relied on when the joystick is neutral
     */
    public Rotation2d rightAngle() {
        return Rotation2d.fromRadians(Math.atan2(rightStickY(), rightStickX()));
    }

    /**
     * Gets the distance of the right joystick from neutral. 
     */
    public double rightHypot() {
        return Math.hypot(rightStickX(), rightStickY());
    }

    /**
     * Gets the value of the left trigger axis, a number in the range 0 (not pressed) to 1 (fully pressed) 
     */
    public double leftTrigger() {
        return controller.getLeftTriggerAxis();
    }
    
    /**
     * Gets the value of the right trigger axis, a number in the range 0 (not pressed) to 1 (fully pressed) 
     */
    public double rightTrigger() {
        return controller.getRightTriggerAxis();
    }
    
    /**
     * Sets the controller to rumble with the given strength (a number in the range [0, 1])
     */
    public void setRumbleStrength(double rumbleStrength) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, rumbleStrength);
    }

    /**
     * Stops the controller rumble. Equivalent to {@link Controller#setRumbleStrength setRumbleStrength(0)}
     */
    public void stopRumble() {
        setRumbleStrength(0);
    }

    /**
     * Returns a command that sets the controller to rumble with the {@link Controller#setRumbleStrength(double) given 
     * strength} while the command is executing and stops the controller rumble when the command ends.
     */
    public Command rumbleCommand(double rumbleStrength) {
        return new StartEndCommand(() -> this.setRumbleStrength(rumbleStrength), this::stopRumble);
    }
}
