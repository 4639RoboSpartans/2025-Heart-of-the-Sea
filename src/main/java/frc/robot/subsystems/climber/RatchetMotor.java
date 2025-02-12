package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents the main motor driving a ratchet gearbox as well as its servo, which controls the ratchet mechanism.
 * The servo will automatically be commanded to disable the ratchet mechanism when the speed is opposite to the
 * ratchet's allowed direction, and enable the mechanism when the speed is in the allowed direction. Because servos
 * do not provide a way to access the angle, disabling the ratchet will always consist of a one-second delay to allow
 * the ratchet to unlock before running the main motor, even if the servo is already almost in position.
 * <p>
 * Make sure to call {@link RatchetMotor#periodic()} in the containing subsystem's {@link Subsystem#periodic()} method.
 * This is what executes the servo logic.
 */
public class RatchetMotor implements MotorController {

    public enum RatchetDirection {
        Forwards(1), Backwards(-1);

        private final int sign;

        RatchetDirection(int sign) {
            this.sign = sign;
        }
    }

    private final MotorController motor;
    // TODO: make private after done testing
    public final Servo servo;
    // TODO: use this
    private final RatchetDirection ratchetDirection;

    private double desiredSpeed = 0;
    private double lastTimeServoWasUp = 0;

    public RatchetMotor(MotorController motor, int servoChannel, RatchetDirection ratchetDirection) {
        this.motor = motor;
        servo = new Servo(servoChannel);
        this.ratchetDirection = ratchetDirection;
    }

    public void periodic() {
        if (this.desiredSpeed * ratchetDirection.sign >= 0) {
            this.servo.set(1.0);
            this.motor.set(this.desiredSpeed);
            this.lastTimeServoWasUp = Timer.getFPGATimestamp();
        } else {
            this.servo.set(0.0);
            if (Timer.getFPGATimestamp() - lastTimeServoWasUp >= 1.0) {
                this.motor.set(this.desiredSpeed);
            } else {
                this.motor.stopMotor();
            }
        }
    }

    @Override
    public void set(double speed) {
        this.desiredSpeed = speed;
    }

    @Override
    public double get() {
        return desiredSpeed;
    }

    @Override
    public void setInverted(boolean isInverted) {
        DriverStation.reportWarning(
            "Setting a RatchetMotor to be inverted does nothing! " +
                "Please use setInverted on the wrapped motor instead " +
                "(and make sure to update ratchetDirection)",
            true
        );
    }

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public void disable() {
        this.desiredSpeed = 0;
        motor.disable();
    }

    @Override
    public void stopMotor() {
        this.desiredSpeed = 0;
        motor.stopMotor();
    }
}
