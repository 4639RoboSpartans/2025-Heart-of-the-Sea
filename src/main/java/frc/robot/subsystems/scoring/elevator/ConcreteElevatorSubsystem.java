package frc.robot.subsystems.scoring.elevator;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Helpers;
import frc.robot.subsystems.scoring.ScoringSuperstructureState;
import frc.robot.subsystems.scoring.constants.ScoringConstants;
import frc.robot.subsystems.scoring.constants.ScoringPIDs;

import static edu.wpi.first.units.Units.*;

public class ConcreteElevatorSubsystem extends ElevatorSubsystem {
    private final TalonFX leftElevator, rightElevator;
    private final MotionMagicVoltage controlRequest;

    private final SysIdRoutine elevatorRoutine;

    private boolean isStateFinished = false;

    public ConcreteElevatorSubsystem() {
        leftElevator = new TalonFX(ScoringConstants.IDs.ElevatorLeftID);
        rightElevator = new TalonFX(ScoringConstants.IDs.ElevatorRightID);
        leftElevator.setNeutralMode(NeutralModeValue.Brake);
        rightElevator.setNeutralMode(NeutralModeValue.Brake);
        var leftConfigurator = leftElevator.getConfigurator();
        TalonFXConfiguration leftConfiguration = new TalonFXConfiguration()
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(ScoringPIDs.elevatorAcceleration.get())
                                .withMotionMagicCruiseVelocity(ScoringPIDs.elevatorVelocity.get()))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ScoringPIDs.elevatorKp.get())
                                .withKI(ScoringPIDs.elevatorKi.get())
                                .withKD(ScoringPIDs.elevatorKd.get())
                );
        leftConfigurator.apply(leftConfiguration);
        rightElevator.setControl(new Follower(ScoringConstants.IDs.ElevatorLeftID, true));
        controlRequest = new MotionMagicVoltage(leftElevator.getPosition().getValueAsDouble());

        elevatorRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(4).per(Second),
                        Volts.of(3),
                        Seconds.of(1.75),
                        (state) -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        output -> leftElevator.setControl(new VoltageOut(output.in(Volts))),
                        null,
                        this
                )
        );
    }

    public void setElevatorState(ScoringSuperstructureState state) {
        isStateFinished = false;
        controlRequest.Position = state.getElevatorAbsolutePosition();
    }

    public void runElevator() {
//        uncomment when down and up positions are set
//        leftElevator.setControl(controlRequest);
        SmartDashboard.putNumber("output", leftElevator.getMotorVoltage().getValueAsDouble());
    }

    protected boolean atState() {
        return MathUtil.isNear(
                controlRequest.Position,
                leftElevator.getPosition().getValueAsDouble(),
                ScoringConstants.ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    public boolean isStateFinished() {
        return isStateFinished;
    }

    public double getCurrentPosition() {
        return leftElevator.getPosition().getValueAsDouble();
    }

    @Override
    public double getTargetPosition() {
        return controlRequest.Position;
    }

    @Override
    public void periodic() {
        runElevator();
        if (atState()) {
            isStateFinished = true;
        }
        SmartDashboard.putBoolean("At State", atState());
    }

    public Command quasistatic(SysIdRoutine.Direction direction) {
        return elevatorRoutine.quasistatic(direction);
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return elevatorRoutine.dynamic(direction);
    }
}
