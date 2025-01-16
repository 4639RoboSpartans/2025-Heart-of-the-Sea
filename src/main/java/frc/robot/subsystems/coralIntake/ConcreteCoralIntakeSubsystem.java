package frc.robot.subsystems.coralIntake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coralIntake.constants.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.constants.CoralIntakePIDs;

public class ConcreteCoralIntakeSubsystem extends CoralIntakeSubsystem implements Subsystem {
    private final TalonFX intakePivot;
    private final SparkFlex intakeMotor;

    private final MotionMagicVoltage pivotControlRequest;

    private CoralIntakeState intakeState = CoralIntakeState.IDLE;

    public ConcreteCoralIntakeSubsystem() {
        intakePivot = new TalonFX(CoralIntakeConstants.IDs.pivotID);
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(
                                        CoralIntakePIDs.pivotVelocity.get()
                                )
                                .withMotionMagicAcceleration(
                                        CoralIntakePIDs.pivotAcceleration.get()
                                )
                )
                .withSlot0(
                        new Slot0Configs()
                                .withKP(
                                        CoralIntakePIDs.pivotKp.get()
                                )
                                .withKI(
                                        CoralIntakePIDs.pivotKi.get()
                                )
                                .withKD(
                                        CoralIntakePIDs.pivotKd.get()
                                )
                );
        intakePivot.getConfigurator().apply(config);
        intakeMotor = new SparkFlex(
                CoralIntakeConstants.IDs.intakeID,
                SparkLowLevel.MotorType.kBrushless
        );
        pivotControlRequest = new MotionMagicVoltage(intakeState.getAbsolutePosition());
    }

    protected void setIntakeState(CoralIntakeState intakeState) {
        this.intakeState = intakeState;
        intakeMotor.set(intakeState.intakeSpeed);
        pivotControlRequest.Position = intakeState.getAbsolutePosition();
        intakePivot.setControl(pivotControlRequest);
    }

    public Trigger atRequestedState() {
        return new Trigger(
                () -> MathUtil.isNear(
                        pivotControlRequest.Position,
                        intakePivot.getPosition(true).getValueAsDouble(),
                        CoralIntakeConstants.PivotConstants.positionTolerance
                )
        );
    }
}
