package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.SwerveModuleConfig;

public class SwerveConstants {

    public static final double TURNING_GEAR_RATIO = 21.4285714286;
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
    public static final double MAX_SPEED = 5.0;

    public static final double DRIVE_P = 0.11;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_KV = 0.109;

    public static final double TURNING_P = 80.0;
    public static final double TURNING_D = 0.5;

    public static final double AUTO_ROTATION_P = 5.0;
    public static final double AUTO_ROTATION_D = 0.0;
    public static final TrapezoidProfile.Constraints AUTO_ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, 4 * Math.PI);

    public static final String FRONT_LEFT_MODULE_NAME = "FRONT_LEFT";
    public static final String FRONT_RIGHT_MODULE_NAME = "FRONT_RIGHT";
    public static final String BACK_LEFT_MODULE_NAME = "BACK_LEFT";
    public static final String BACK_RIGHT_MODULE_NAME = "BACK_RIGHT";

    // Hardware IDs
    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_TURNING_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_ID = 7;
    public static final int FRONT_RIGHT_TURNING_ID = 8;
    public static final int BACK_LEFT_DRIVE_ID = 1;
    public static final int BACK_LEFT_TURNING_ID = 2;
    public static final int BACK_RIGHT_DRIVE_ID = 5;
    public static final int BACK_RIGHT_TURNING_ID = 6;

    public static final int FRONT_LEFT_ENCODER_ID = 12;
    public static final int FRONT_RIGHT_ENCODER_ID = 14;
    public static final int BACK_LEFT_ENCODER_ID = 11;
    public static final int BACK_RIGHT_ENCODER_ID = 13;

    // Inversions
    public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    public static final boolean BACK_RIGHT_DRIVE_INVERTED = false;

    public static final boolean FRONT_LEFT_TURNING_INVERTED = false;
    public static final boolean FRONT_RIGHT_TURNING_INVERTED = false;
    public static final boolean BACK_LEFT_TURNING_INVERTED = false;
    public static final boolean BACK_RIGHT_TURNING_INVERTED = false;


    public static final double FRONT_LEFT_OFFSET_X = Units.inchesToMeters(11.0);
    public static final double FRONT_LEFT_OFFSET_Y = Units.inchesToMeters(11.0);

    public static final double FRONT_RIGHT_OFFSET_X = Units.inchesToMeters(11.0);
    public static final double FRONT_RIGHT_OFFSET_Y = Units.inchesToMeters(-11.0);

    public static final double BACK_LEFT_OFFSET_X = Units.inchesToMeters(-11.0);
    public static final double BACK_LEFT_OFFSET_Y = Units.inchesToMeters(11.0);

    public static final double BACK_RIGHT_OFFSET_X = Units.inchesToMeters(-11.0);
    public static final double BACK_RIGHT_OFFSET_Y = Units.inchesToMeters(-11.0);

    // Module Positions (Translation2d)
    public static final Translation2d FRONT_LEFT_MODULE_POSE = new Translation2d(FRONT_LEFT_OFFSET_X, FRONT_LEFT_OFFSET_Y);
    public static final Translation2d FRONT_RIGHT_MODULE_POSE = new Translation2d(FRONT_RIGHT_OFFSET_X, FRONT_RIGHT_OFFSET_Y);
    public static final Translation2d BACK_LEFT_MODULE_POSE = new Translation2d(BACK_LEFT_OFFSET_X, BACK_LEFT_OFFSET_Y);
    public static final Translation2d BACK_RIGHT_MODULE_POSE = new Translation2d(BACK_RIGHT_OFFSET_X, BACK_RIGHT_OFFSET_Y);

    // Absolute Encoder Offsets
    public static final double FRONT_LEFT_ENCODER_OFFSET = 149.854;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 320.01;
    public static final double BACK_LEFT_ENCODER_OFFSET = 228.164;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 179.561;

    // --- Module Configurations ---

    public static final SwerveModuleConfig FRONT_LEFT_CONFIG = new SwerveModuleConfig(
            FRONT_LEFT_MODULE_NAME,
            FRONT_LEFT_DRIVE_ID,
            FRONT_LEFT_TURNING_ID,
            FRONT_LEFT_ENCODER_ID,
            FRONT_LEFT_DRIVE_INVERTED,
            FRONT_LEFT_TURNING_INVERTED,
            FRONT_LEFT_MODULE_POSE,
            FRONT_LEFT_ENCODER_OFFSET
    );

    public static final SwerveModuleConfig FRONT_RIGHT_CONFIG = new SwerveModuleConfig(
            FRONT_RIGHT_MODULE_NAME,
            FRONT_RIGHT_DRIVE_ID,
            FRONT_RIGHT_TURNING_ID,
            FRONT_RIGHT_ENCODER_ID,
            FRONT_RIGHT_DRIVE_INVERTED,
            FRONT_RIGHT_TURNING_INVERTED,
            FRONT_RIGHT_MODULE_POSE,
            FRONT_RIGHT_ENCODER_OFFSET
    );

    public static final SwerveModuleConfig BACK_LEFT_CONFIG = new SwerveModuleConfig(
            BACK_LEFT_MODULE_NAME,
            BACK_LEFT_DRIVE_ID,
            BACK_LEFT_TURNING_ID,
            BACK_LEFT_ENCODER_ID,
            BACK_LEFT_DRIVE_INVERTED,
            BACK_LEFT_TURNING_INVERTED,
            BACK_LEFT_MODULE_POSE,
            BACK_LEFT_ENCODER_OFFSET
    );

    public static final SwerveModuleConfig BACK_RIGHT_CONFIG = new SwerveModuleConfig(
            BACK_RIGHT_MODULE_NAME,
            BACK_RIGHT_DRIVE_ID,
            BACK_RIGHT_TURNING_ID,
            BACK_RIGHT_ENCODER_ID,
            BACK_RIGHT_DRIVE_INVERTED,
            BACK_RIGHT_TURNING_INVERTED,
            BACK_RIGHT_MODULE_POSE,
            BACK_RIGHT_ENCODER_OFFSET
    );

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            FRONT_LEFT_MODULE_POSE,
            FRONT_RIGHT_MODULE_POSE,
            BACK_LEFT_MODULE_POSE,
            BACK_RIGHT_MODULE_POSE
    );
}