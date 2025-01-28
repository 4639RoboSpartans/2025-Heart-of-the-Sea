package frc.robot.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.ScoringSuperstructure;

import static edu.wpi.first.units.Units.Inches;


public class RobotSim extends SubsystemBase {
    public static final double height = 120;
    public static final double width = 28.5;
    public static final java.awt.Color currentColor = new java.awt.Color(16, 125, 215);
    public static final java.awt.Color targetColor = new java.awt.Color(0, 178, 99);

    public static final Distance hopperLength = Inches.of(14);

    public static final Translation2d origin =
            new Translation2d(Units.inchesToMeters(width / 2), 0.0);

    public static final Mechanism2d currentView =
            new Mechanism2d(Units.inchesToMeters(width) - 6.25, Units.inchesToMeters(height));

    public static final MechanismRoot2d elevatorRoot =
            currentView.getRoot("Current Elevator Root", origin.getX(), origin.getY());

    public static final MechanismLigament2d currentElevatorLigament =
            elevatorRoot.append(
                    new MechanismLigament2d(
                            "Current Elevator Ligament",
                            ScoringSuperstructure.getInstance().getCurrentElevatorLength().in(Inches),
                            90,
                            2,
                            new Color8Bit(currentColor.getRed(), currentColor.getGreen(), currentColor.getBlue())
                    )
            );

    public static final MechanismLigament2d currentHopperLigament =
            currentElevatorLigament.append(
                    new MechanismLigament2d(
                            "Current Hopper Ligament",
                            hopperLength.in(Inches),
                            90,
                            2,
                            new Color8Bit(currentColor.getRed(), currentColor.getGreen(), currentColor.getBlue())
                    )
            );

    public static final MechanismLigament2d targetElevatorLigament =
            elevatorRoot.append(
                    new MechanismLigament2d(
                            "Target Elevator Ligament",
                            ScoringSuperstructure.getInstance().getTargetElevatorLength().in(Inches),
                            90,
                            2,
                            new Color8Bit(targetColor.getRed(), targetColor.getGreen(), targetColor.getBlue())
                    )
            );

    public static final MechanismLigament2d targetHopperLigament =
            elevatorRoot.append(
                    new MechanismLigament2d(
                            "Target Hopper Ligament",
                            hopperLength.in(Inches),
                            90,
                            2,
                            new Color8Bit(targetColor.getRed(), targetColor.getGreen(), targetColor.getBlue())
                    )
            );


    public RobotSim() {
        SmartDashboard.putData("Mechanism View", RobotSim.currentView);
        currentView.setBackgroundColor(new Color8Bit(Color.kLightGray));
    }

    @Override
    public void periodic() {
        currentElevatorLigament.setLength(
                ScoringSuperstructure.getInstance().getCurrentElevatorLength().in(Inches)
        );
        currentHopperLigament.setAngle(
                ScoringSuperstructure.getInstance().getCurrentWristRotation()
        );

        targetElevatorLigament.setLength(
                ScoringSuperstructure.getInstance().getTargetElevatorLength().in(Inches)
        );
        targetHopperLigament.setAngle(
                ScoringSuperstructure.getInstance().getTargetWristRotation()
        );
    }
}