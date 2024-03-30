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
                 * 
                 * 
                 */
                // do this without converting to radians and stuff
                SHOOTER_TUNING.put(new InterpolatingDouble(1.0),
                                new Vector2(60, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(1.25),
                                new Vector2(57.5, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(1.5),
                                new Vector2(53, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(1.75),
                                new Vector2(49.5, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(2.0),
                                new Vector2(45, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(2.22),
                                new Vector2(43.5, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(2.5),
                                new Vector2(42.75, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(2.75),
                                new Vector2(41.25, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(3.0),
                                new Vector2(40.75, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(3.25),
                                new Vector2(40.4, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(3.5),
                                new Vector2(40.3, 6000));
                SHOOTER_TUNING.put(new InterpolatingDouble(4.0),
                                new Vector2(40, 6000));

        }

        public ShooterRegression() {

        }

        public Vector2 getAngleAndRPM(double distance) {
                Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(distance));
                return angleAndSpeed;
        }

        public double getAngle(double distance) {
                Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(distance));
                return angleAndSpeed.x - 2.5 - 2;
        }

        public double getRPM(double distance) {
                Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(distance));
                return angleAndSpeed.y;
        }

}
