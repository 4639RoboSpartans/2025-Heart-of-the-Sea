package frc.robot.subsystems.algaeIntake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.algaeIntake.constants.AlgaeIntakeConstants;

public class ConcreteAlgaeIntakeSubsystem extends AlgaeIntakeSubsystem {
    private final SparkFlex pivotMotor;
    private final SparkMax intakeMotor;

    public ConcreteAlgaeIntakeSubsystem() {
        pivotMotor = new SparkFlex(
            AlgaeIntakeConstants.IDs.pivotID,
            SparkLowLevel.MotorType.kBrushless
        );
        intakeMotor = new SparkMax(
            AlgaeIntakeConstants.IDs.intakeID,
            SparkLowLevel.MotorType.kBrushless
        );
    }

    @Override
    protected void setPivotSpeed(double percent) {
        pivotMotor.set(percent * AlgaeIntakeConstants.PIVOT_MAX_SPEED);
    }

    @Override
    protected void setIntakeSpeed(double percent) {
        intakeMotor.set(percent * AlgaeIntakeConstants.INTAKE_SPEED);
    }
}
