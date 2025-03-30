package frc.robot.subsystems.climber;

import frc.lib.tunable.TunableNumber;

public class ClimberConstants {
    public static final int CLIMBER_ID = 50;
    public static final TunableNumber climberSpeed = new TunableNumber("climber/climberSpeed").withDefaultValue(-0.25);

    public static final int SERVO_ID = 0;

    public static final class Setpoints {
        public static final TunableNumber holdingPosition = new TunableNumber("servo/holdingPosition").withDefaultValue(1.0);
        public static final TunableNumber dropPosition = new TunableNumber("servo/dropPosition").withDefaultValue(-1.0);

        public static final TunableNumber climbPosition = new TunableNumber("climb/Climb Position").withDefaultValue(0.6935);
        public static final TunableNumber stowedPosition = new TunableNumber("climb/Stowed Position").withDefaultValue(0);
        public static final TunableNumber readyToClimbPosition = new TunableNumber("climb/Ready to Climb Position").withDefaultValue(0);

        public static final TunableNumber encoderZero = new TunableNumber("climb/Encoder Zero").withDefaultValue(0);
    }

    public static final class PIDs {
        public static final TunableNumber climbSpeed = new TunableNumber("climb/Climb Speed").withDefaultValue(0.25);
    }
}