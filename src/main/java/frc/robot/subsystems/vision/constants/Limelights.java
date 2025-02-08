package frc.robot.subsystems.vision.constants;

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
