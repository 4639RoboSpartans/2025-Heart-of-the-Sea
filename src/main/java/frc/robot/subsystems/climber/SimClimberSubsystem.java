package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.annotation.PackagePrivate;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Objects;

public class SimClimberSubsystem extends AbstractClimberSubsystem{
    ClimberState state = ClimberState.STOWED;
    private final SingleJointedArmSim climberSim;
    private final DCMotor gearbox = DCMotor.getVex775Pro(1);

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
    double getEncoderPosition() {
        return Rotation2d.fromRadians(climberSim.getAngleRads()).getRotations();
    }

    @Override
    public void periodic() {
    }
}
