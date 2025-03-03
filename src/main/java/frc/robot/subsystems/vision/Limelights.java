package frc.robot.subsystems.vision;

import java.util.Collection;
import java.util.HashSet;
import java.util.Objects;

public class Limelights {
    public static final Limelight LEFT = constructLimelight("limelight");
    public static final Limelight RIGHT = constructLimelight("limelight-slhs");

    public static Collection<Limelight> all = new HashSet<Limelight>();

    private static Limelight constructLimelight(String name){
       var limelight = new Limelight(name);
       Objects.requireNonNull(all).add(limelight);
       return limelight;
    }
}
