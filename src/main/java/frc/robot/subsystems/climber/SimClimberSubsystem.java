package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.annotation.PackagePrivate;

import java.util.Objects;

public class SimClimberSubsystem extends AbstractClimberSubsystem{
    ClimberState state = ClimberState.IDLE;
    private final SingleJointedArmSim climberSim;
    private final DCMotor gearbox = DCMotor.getVex775Pro(1);

    private static volatile SimClimberSubsystem instance;

    public static synchronized SimClimberSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, SimClimberSubsystem::new);
    }

    public SimClimberSubsystem() {
        init();
        climberSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        gearbox,
                        SingleJointedArmSim.estimateMOI(0.25, 5),
                        210.0
                ),
                gearbox,
                210.0,
                0.25,
                Math.PI/6,
                Math.PI/2,
                false,
                Math.PI/6
        );
    }

    @Override
    void setClimberSpeed(double speed) {
        SmartDashboard.putNumber("climber speed", speed);
    }

    @Override
    ClimberState getClimberState() {
        return state;
    }

    @Override
    void setClimberState(ClimberState state) {
        this.state = state;
    }

    @Override
    void setServoPosition(double servoPosition) {
        SmartDashboard.putNumber("servo position", servoPosition);
    }

    @Override
    double getServoPosition() {
        return SmartDashboard.getNumber("servo position", 0);
    }

    @Override
    public void periodic() {
    }
}
