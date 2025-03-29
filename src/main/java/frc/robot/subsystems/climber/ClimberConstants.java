package frc.robot.subsystems.climber;

import frc.lib.tunable.TunableNumber;

public class ClimberConstants {
    public static final int CLIMBER_ID = 50;
    public static final TunableNumber climberSpeed = new TunableNumber("climber/climberSpeed").withDefaultValue(0.5);

    public static final int SERVO_ID = 0;

    public static final class ServoSetpoints {
        public static final TunableNumber holdingPosition = new TunableNumber("servo/holdingPosition").withDefaultValue(1.0);
        public static final TunableNumber dropPosition = new TunableNumber("servo/dropPosition").withDefaultValue(0.0);
    }

    public static final class ClimberSetpoints {
        public static final TunableNumber outPosition = new TunableNumber("climber/outPosition").withDefaultValue(0.0);
        public static final TunableNumber inPosition = new TunableNumber("climber/outPosition").withDefaultValue(0.0);
    }
}
