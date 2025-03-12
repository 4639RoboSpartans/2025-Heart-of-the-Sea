package frc.lib.led;

import edu.wpi.first.wpilibj.util.Color;

public class OceanWaveLEDPattern implements LEDPattern{
    private final int lengthOfFirstSegment;
    private final int lengthOfSecondSegment;
    private final int lengthOfDeadZone;
    private final double speed;

    private final Color WSC; //wetSandColor
    private final Color WC; //whiteColor
    private final Color CC; //cyanColor
    private final Color BC; //blueColor

    private Color[] map;

    public OceanWaveLEDPattern(int lengthOfFirstSegment, int lengthOfSecondSegment, int lengthOfDeadZone){
        this(lengthOfFirstSegment, lengthOfSecondSegment, lengthOfDeadZone, 1);
    }

    public OceanWaveLEDPattern(int lengthOfFirstSegment, int lengthOfSecondSegment, int lengthOfDeadZone, double speed){
        this.lengthOfFirstSegment = lengthOfFirstSegment;
        this.lengthOfSecondSegment = lengthOfSecondSegment;
        this.lengthOfDeadZone = lengthOfDeadZone;
        this.speed = speed;

        WSC = new Color(174, 143, 96);
        WC = new Color("#03738C");
        CC = new Color("#038C73");
        BC = new Color(128, 128, 128); //warm white(if needed): 255, 200, 150

        map = new Color[lengthOfFirstSegment+lengthOfDeadZone+lengthOfSecondSegment];
    }

    @Override
    public Color get(int led, double time){
        return null;
    }

    public void runColor(double time){
        int deepWaterLength = (int)Math.floor(lengthOfFirstSegment*.4);
        int shallowWaterLength = (int)Math.floor(lengthOfFirstSegment*.4);
        int sandLength = lengthOfFirstSegment - deepWaterLength - shallowWaterLength;

        int cCyanPoint = (int)Math.floor(lengthOfFirstSegment*.65);

        int roundedTime = (int)Math.floor(time * speed);
        int WaveFrontPosition = roundedTime % 25;
        int sandFrontPos = 23 - WaveFrontPosition;

        for(int i = 0; i < lengthOfFirstSegment; i++){
            if(WaveFrontPosition < 22){
                if(i >= lengthOfFirstSegment - sandLength){
                    map[i] = WSC;
                }
                if((i >= cCyanPoint && i < WaveFrontPosition - 2 && i < lengthOfFirstSegment) || (i >= deepWaterLength && i < deepWaterLength + shallowWaterLength)){
                    map[i] = CC;
                }
                if((i < cCyanPoint && i < WaveFrontPosition - 2) || i < deepWaterLength){
                    map[i] = BC;
                }
                if(i == WaveFrontPosition || i == WaveFrontPosition - 1){
                    map[i] = WC;
                }
            }
            else{
                if(i < lengthOfFirstSegment - shallowWaterLength - sandFrontPos){
                    map[i] = BC;
                }
                else if(i >= lengthOfFirstSegment - shallowWaterLength - sandFrontPos && i < lengthOfFirstSegment - sandFrontPos){
                    map[i] = CC;
                }
                else{
                    map[i] = WSC;
                }
            }
        }
    }
}
