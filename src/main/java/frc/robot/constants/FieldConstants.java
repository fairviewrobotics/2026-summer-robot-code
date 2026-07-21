package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Bounds;

public class FieldConstants {

    public static final double FIELD_BORDER_MARGIN_METERS = 0.5;
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.069326;

    public static final double BALL_HEIGHT_METERS = Units.inchesToMeters(6.0);

    public static final Pose3d BLUE_HUB_POSE3D = new Pose3d(4.62534,4.034663,1.822, Rotation3d.kZero);
    public static final Pose3d RED_HUB_POSE3D = new Pose3d(4.62534 + 7.2898,4.034663,1.822, Rotation3d.kZero);
    public static final Pose2d BLUE_PASS_RIGHT_POSE = new Pose2d(2.4, 0.8, Rotation2d.kZero);
    public static final Pose2d BLUE_PASS_LEFT_POSE = new Pose2d(2.4, 7.2, Rotation2d.kZero);

    public static final Pose2d BLUE_TRENCH_LEFT = new Pose2d(4.42, 7.415, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_TRENCH_LEFT_INTAKE_START = new Pose2d(7.715, 7.415, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_TRENCH_LEFT_INTAKE_END = new Pose2d(7.715, 4.468, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_BUMP_LEFT_DEPART = new Pose2d(7.78, 5.584, new Rotation2d(-0.4018));
    public static final Pose2d BLUE_BUMP_LEFT_RETURN = new Pose2d(5.596, 5.584, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_LEFT_SHOOT_POSE = new Pose2d(3.138, 5.584, new Rotation2d(2.291));

    public static final Bounds TRENCH_BOUNDS = new Bounds(4, 5.25, 0, 8.5);

}