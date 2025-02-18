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
                    .withKP(elevatorKp.get())
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

        setTargetExtensionProportion(getCurrentProportion());
    }

    @Override
    public double getCurrentProportion() {
        return ElevatorConstants.ProportionToPosition.convertBackwards(
            elevatorMotor.getPosition(true).getValueAsDouble()
        );
    }

    @Override
    public void periodic() {
        if (isManualControlEnabled) {
            double outputVoltage = Controls.Operator.ManualControlElevator.getAsDouble() * 4;

            // If we are moving downwards, go slower for safety
            if (outputVoltage < 0) outputVoltage /= 2.;

            // Prevent movement if we are too high or low
            double ELEVATOR_MANUAL_ENDPOINT_LIMIT = -0.05;
            if (outputVoltage > 0 && getCurrentProportion() > 1 - ELEVATOR_MANUAL_ENDPOINT_LIMIT) {
                outputVoltage = 0;
            }
            if (outputVoltage < 0 && getCurrentProportion() < ELEVATOR_MANUAL_ENDPOINT_LIMIT) {
                outputVoltage = 0;
            }

            // Add the kG term times a fudge factor to keep the elevator stationary when no input is given
            outputVoltage += elevatorKg.get() * 1.2;

            elevatorMotor.setControl(new VoltageOut(outputVoltage));
        } else {
            elevatorMotor.setControl(new MotionMagicVoltage(getTargetPosition()));
        }

        SmartDashboard.putNumber("Elevator Proportion", ElevatorConstants.ProportionToPosition.convertBackwards(getCurrentPosition()));
        SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
        SmartDashboard.putBoolean("Elevator at Target", isAtTarget());
    }

    @Override
    public void setRawMotorVoltage(Voltage voltage) {
        elevatorMotor.setControl(new VoltageOut(voltage));
    }
}
