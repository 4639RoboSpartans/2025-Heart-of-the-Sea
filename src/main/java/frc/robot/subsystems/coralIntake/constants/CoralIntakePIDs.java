package frc.robot.subsystems.coralIntake.constants;

import frc.lib.TunableNumber;

public class CoralIntakePIDs {
    public static final TunableNumber pivotKp = new TunableNumber("pivotKp");
    public static final TunableNumber pivotKi = new TunableNumber("pivotKi");
    public static final TunableNumber pivotKd = new TunableNumber("pivotKd");
    public static final TunableNumber pivotVelocity = new TunableNumber("pivotVelocity");
    public static final TunableNumber pivotAcceleration = new TunableNumber("pivotAcceleration");

    static {
        pivotKp.setDefault(0.1);
        pivotKi.setDefault(0.1);
        pivotKd.setDefault(0.1);
        pivotVelocity.setDefault(0.1);
        pivotAcceleration.setDefault(0.1);
    }
}
