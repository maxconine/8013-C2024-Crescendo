package com.team8013.frc2024.regressions;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.Vector2;

public class ShooterRegression {
    
    private static final InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();

    static {

        /*
            * EXAMPLE:
            * 
            * distance, angle, rpm
            * 
            * do this without converting to radians and stuff
            SHOOTER_TUNING.put(new InterpolatingDouble(1.0),
                    new Vector2(Math.toRadians(7.0), Units.rotationsPerMinuteToRadiansPerSecond(1725)));
            SHOOTER_TUNING.put(new InterpolatingDouble(1.5),
                    new Vector2(Math.toRadians(9.5), Units.rotationsPerMinuteToRadiansPerSecond(1725)));
            SHOOTER_TUNING.put(new InterpolatingDouble(2.0),
                    new Vector2(Math.toRadians(11.5), Units.rotationsPerMinuteToRadiansPerSecond(1750)));
            SHOOTER_TUNING.put(new InterpolatingDouble(2.5),
                    new Vector2(Math.toRadians(13.5), Units.rotationsPerMinuteToRadiansPerSecond(1825)));
            SHOOTER_TUNING.put(new InterpolatingDouble(3.0),
                    new Vector2(Math.toRadians(16.0), Units.rotationsPerMinuteToRadiansPerSecond(1900)));
            SHOOTER_TUNING.put(new InterpolatingDouble(3.5),
                    new Vector2(Math.toRadians(18.5), Units.rotationsPerMinuteToRadiansPerSecond(2000)));
            SHOOTER_TUNING.put(new InterpolatingDouble(4.0),
                    new Vector2(Math.toRadians(21.5), Units.rotationsPerMinuteToRadiansPerSecond(2100)));
            SHOOTER_TUNING.put(new InterpolatingDouble(4.5),
                    new Vector2(Math.toRadians(24.0), Units.rotationsPerMinuteToRadiansPerSecond(2175)));
            SHOOTER_TUNING.put(new InterpolatingDouble(5.0),
                    new Vector2(Math.toRadians(25.5), Units.rotationsPerMinuteToRadiansPerSecond(2275)));
            SHOOTER_TUNING.put(new InterpolatingDouble(5.5),
                    new Vector2(Math.toRadians(27.5), Units.rotationsPerMinuteToRadiansPerSecond(2400)));
            SHOOTER_TUNING.put(new InterpolatingDouble(6.0),
                    new Vector2(Math.toRadians(30.0), Units.rotationsPerMinuteToRadiansPerSecond(2475)));
            */
        
            
    }
    
    public ShooterRegression(){}



    public Vector2 getAngleAndRPM(double distance){
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(distance));
        return angleAndSpeed;
    }

    public double getAngle(double distance){
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(distance));
        return angleAndSpeed.x;
    }

    public double getRPM(double distance){
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(distance));
        return angleAndSpeed.y;
    }


}
