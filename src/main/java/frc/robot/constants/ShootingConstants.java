package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShootingConstants {

    public static final int LEFT_SHOOTER_MOTOR_ID = 23;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 22;

    public static final int HOOD_MOTOR_ID = 0;

    public static final double HOOD_MAX_ANGLE_DEGREES = 90.0;
    public static final double HOOD_MIN_ANGLE_DEGREES = 0.0;

    public static final double DEFAULT_KP = 0.1;
    public static final double DEFAULT_KI = 0.0;
    public static final double DEFAULT_KD = 0.0;

    public static final double DEFAULT_KV = 0.115;
    public static final double DEFAULT_KS = 0.2;


    public static final double SHOOTER_RPM = 2500;
    public static final double SHOOTER_TOLERANCE_RPM = 400;

    // Ratio so 1 mechanism rotation = full hood sector in radians
    public static final double HOOD_CONVERSION_FACTOR = 190.0 / (22.0 * Math.PI);

    // Just make it spun up the entire time unc
    public static final double AUTO_SHOOTER_TIMEOUT_SECONDS = 2.0;

}