package frc.robot.subsystems.scoring.elevator;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;

import static frc.robot.subsystems.scoring.constants.ScoringConstants.ElevatorConstants;
import static frc.robot.subsystems.scoring.constants.ScoringConstants.IDs;
import static frc.robot.subsystems.scoring.constants.ScoringPIDs.*;

public class ConcreteElevatorSubsystem extends ElevatorSubsystem {
    private final TalonFX elevatorMotor;
    private final MotionMagicVoltage controlRequest;

    private boolean isStateFinished = false;

    @SuppressWarnings("resource")
    public ConcreteElevatorSubsystem() {
        TalonFX leftElevatorMotor = new TalonFX(IDs.ElevatorLeftID, IDs.ElevatorCANBusName);
        TalonFX rightElevatorMotor = new TalonFX(IDs.ElevatorRightID, IDs.ElevatorCANBusName);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        var leftConfigurator = leftElevatorMotor.getConfigurator();
        var rightConfigurator = rightElevatorMotor.getConfigurator();
        TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(elevatorAcceleration.get())
                    .withMotionMagicCruiseVelocity(elevatorVelocity.get())
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(elevatorKp.get())
                    .withKI(elevatorKi.get())
                    .withKD(elevatorKd.get())
                    .withKA(elevatorKa.get())
                    .withKS(elevatorKs.get())
                    .withKV(elevatorKv.get())
                    .withKG(elevatorKg.get())
                    .withGravityType(GravityTypeValue.Elevator_Static)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(
                        30
                    )
            );

        leftConfigurator.apply(configuration);
        rightConfigurator.apply(configuration);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        leftElevatorMotor.setControl(new Follower(IDs.ElevatorRightID, true));

        elevatorMotor = rightElevatorMotor;

        controlRequest = new MotionMagicVoltage(leftElevatorMotor.getPosition().getValueAsDouble());
    }

    /**
     * Gets the current proprtion that the elevator is at
     * 
     * @return elevator proportion as double
     */
    @Override
    public double getCurrentProportion() {
        return ElevatorConstants.ProportionToPosition.convertBackwards(
            elevatorMotor.getPosition(true).getValueAsDouble()
        );
    }

    /**
     * Checks if the elevator is at its desired state
     * 
     * @returns if elevator is at desired state as boolean
     */
    @Override
    public boolean isElevatorStateFinished() {
        return isStateFinished;
    }

    /**
     * Checks if the elevator is at its target position
     * 
     * @return if elevator is at position as boolean
     */
    @Override
    public boolean isAtTarget() {
        return MathUtil.isNear(
            controlRequest.Position,
            getCurrentPosition(),
            ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    /**
     * Sets the state of the elevator and its target position
     */
    @Override
    public void updateElevatorState(ScoringSuperstructureState state) {
        this.state = state;
        isStateFinished = false;
        controlRequest.Position = getTargetPosition();
    }

    /**
     * Runs the Elevator
     */
    @Override
    public void runElevator() {
        elevatorMotor.setControl(controlRequest);
        SmartDashboard.putNumber("output", elevatorMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (isAtTarget()) {
            isStateFinished = true;
        }
        SmartDashboard.putNumber("Elevator Proportion", ElevatorConstants.ProportionToPosition.convertBackwards(getCurrentPosition()));
        SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
        SmartDashboard.putBoolean("Elevator is at Target", isAtTarget());
        SignalLogger.writeDouble("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Velocity", elevatorMotor.getVelocity().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Acceleration", elevatorMotor.getAcceleration().getValueAsDouble());
        SignalLogger.writeDouble("Elevator Voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
    }

    /**
     * Sets the elevator motors as a specific Voltage
     * 
     * @param Voltage the voltage elevator is being set to
     */
    @Override
    public void setRawMotorVoltage(Voltage voltage) {
        elevatorMotor.setControl(new VoltageOut(voltage));
    }
}
