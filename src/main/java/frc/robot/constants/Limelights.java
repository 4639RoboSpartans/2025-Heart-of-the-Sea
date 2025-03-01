package frc.robot.constants;

public enum Limelights {
    LIMELIGHT("limelight"),
    LIMELIGHT2("limelight-slhs");

    final String name;
    Limelights(String name){
        this.name = name;
    }

    public String getName(){
        return name;
    }
}
