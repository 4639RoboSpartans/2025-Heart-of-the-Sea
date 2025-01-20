package frc.robot.constants;

public final class IDs {
    private IDs() {}

    public static final int LEFT_CLIMBER_MOTOR = 13;
    public static final int RIGHT_CLIMBER_MOTOR = 14;

    public enum Limelights {
        LIMELIGHT_1("limelight");

        private final String name;
        Limelights(String name) {
            this.name = name;
        }
        public String getName() {
            return name;
        }
    }
}
