package frc.robot.constants;

public enum Limelights {
    LIMELIGHT("limelight-left"),
    LIMELIGHT2("limelight-right");

    final String name;
    Limelights(String name){
        this.name = name;
    }

    public String getName(){
        return name;
    }
}
