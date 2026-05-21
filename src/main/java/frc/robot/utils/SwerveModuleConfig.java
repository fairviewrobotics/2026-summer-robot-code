package frc.robot.utils;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public record SwerveModuleConfig(
    String name,
    int driveMotorID,
    int turnMotorID,
    int encoderID,
    boolean driveInverted,
    boolean turnInverted,
    Translation2d modulePose,
    double absoluteEncoderOffset
) {}
