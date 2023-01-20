package frc.robot.subsystems;

import java.lang.Math;

public class SwerveDrive {

        // use measured values
        
        public final double L = 9;
        public final double W = 9;

        public void drive (double x1, double y1, double x2) {
                double r = Math.sqrt ((L * L) + (W * W));
                y1 *= -1;

                double a = x1 - x2 * (L / r);
                double b = x1 + x2 * (L / r);
                double c = y1 - x2 * (W / r);
                double d = y1 + x2 * (W / r);

                double backRightSpeed = Math.sqrt ((a * a) + (d * d));
                double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
                double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
                double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

                // remove the multiplication by 180 to go from degrees to a range from -1 to 1

                double backRightAngle = Math.atan2 (a, d) / Math.PI;
                double backLeftAngle = Math.atan2 (a, c) / Math.PI;
                double frontRightAngle = Math.atan2 (b, d) / Math.PI;
                double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
        }
}
