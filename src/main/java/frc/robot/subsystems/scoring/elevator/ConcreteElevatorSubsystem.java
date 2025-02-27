package frc.robot.subsystems.scoring.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Controls;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.IDs;
import static frc.robot.subsystems.scoring.constants.ScoringPIDs.*;

public class ConcreteElevatorSubsystem extends AbstractElevatorSubsystem {
    private final TalonFX elevatorMotor;
    private double measuredExtensionFractionOffset = 0;

    @SuppressWarnings("resource")
    public ConcreteElevatorSubsystem() {
        // Instantiate motors
        TalonFX leftElevatorMotor = new TalonFX(IDs.ElevatorLeftID, IDs.ElevatorCANBusName);
        TalonFX rightElevatorMotor = new TalonFX(IDs.ElevatorRightID, IDs.ElevatorCANBusName);

        // Set up motor configuration
        TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(elevatorAcceleration.get())
                    .withMotionMagicCruiseVelocity(elevatorVelocity.get())
            ).withSlot0(
                new Slot0Configs()
                    .withKP(elevatorKp.get() / 2.)
                    .withKI(elevatorKi.get())
                    .withKD(elevatorKd.get())
                    .withKA(elevatorKa.get())
                    .withKS(elevatorKs.get())
                    .withKV(elevatorKv.get())
                    .withKG(elevatorKg.get())
                    .withGravityType(GravityTypeValue.Elevator_Static)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(
                        30
                    )
            );

        // Apply configuration
        leftElevatorMotor.getConfigurator().apply(configuration);
        rightElevatorMotor.getConfigurator().apply(configuration);

        // Set neutral mode to brake
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        // Set left motor to follow right motor and use right elevator motor as master motor
        leftElevatorMotor.setControl(new Follower((elevatorMotor = rightElevatorMotor).getDeviceID(), true));

        setTargetExtensionFraction(getCurrentExtensionFraction());
    }

    @Override
    public double getCurrentExtensionFraction() {
        return ElevatorConstants.ProportionToPosition.convertBackwards(
            elevatorMotor.getPosition(true).getValueAsDouble()
        ) + measuredExtensionFractionOffset;
    }

    @Override
    public void periodic() {
        if (isManualControlEnabled) {
            double outputVoltage = Controls.Operator.ManualControlElevator.getAsDouble() * 4;

            // If we are moving downwards, go slower for safety
            if (outputVoltage < 0) outputVoltage /= 2.;

            // Prevent movement if we are too high or low
            double ELEVATOR_MANUAL_ENDPOINT_LIMIT = -0.05;
            if (outputVoltage > 0 && getCurrentExtensionFraction() > 1 - ELEVATOR_MANUAL_ENDPOINT_LIMIT) {
                outputVoltage = 0;
            }
            if (outputVoltage < 0 && getCurrentExtensionFraction() < ELEVATOR_MANUAL_ENDPOINT_LIMIT) {
                outputVoltage = 0;
            }

            // Add the kG term times a fudge factor to keep the elevator stationary when no input is given
            outputVoltage += elevatorKg.get() * 1.2;

            elevatorMotor.setControl(new VoltageOut(outputVoltage));
        } else {
            elevatorMotor.setControl(new MotionMagicVoltage(
                ElevatorConstants.ProportionToPosition.convert(
                    // In measurement, we have measured = raw + offset,
                    // so here we must have raw = calculated - offset
                    getTargetExtensionFraction() - measuredExtensionFractionOffset
                )
            ));
        }

        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Motor RPS", elevatorMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void setRawMotorVoltage(Voltage voltage) {
        elevatorMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public boolean isPhysicallyStopped() {
        //TODO: evaluate numbers and figure out the conditions for stopping the command
        return Math.abs(elevatorMotor.getTorqueCurrent().getValueAsDouble()) > 20 && Math.abs(elevatorMotor.getVelocity().getValueAsDouble()) <= 0.2;
    }

    @Override
    public void resetCurrentExtensionFractionTo(double extensionFraction) {
        double currentError = getCurrentExtensionFraction() - extensionFraction;
        measuredExtensionFractionOffset -= currentError;
        setTargetExtensionFraction(extensionFraction);
    }
}
