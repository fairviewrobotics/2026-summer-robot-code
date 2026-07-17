package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class RotateToAngle extends Command {

    private ProfiledPIDController thetaController;
    private Swerve swerveSubsystem;
    private double thetaErrorAbs;
    private Rotation2d targetRotation;
    private Supplier<Rotation2d> targetRotationSupplier;

    private static final String AUTO_ROTATION_P_KEY = "DriveToPoint/AutoRotationP";

    public RotateToAngle(
            Swerve swerveSubsystem,
            Supplier<Rotation2d> targetRotationSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetRotationSupplier = targetRotationSupplier;

        Preferences.initDouble(AUTO_ROTATION_P_KEY, SwerveConstants.AUTO_ROTATION_P);

        this.thetaController =
                new ProfiledPIDController(
                        Preferences.getDouble(AUTO_ROTATION_P_KEY, SwerveConstants.AUTO_ROTATION_P),
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                SwerveConstants.MAX_ANGULAR_SPEED,
                                SwerveConstants.MAX_ANGULAR_SPEED * 2.0),
                        0.02);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveSubsystem);
    }

    public RotateToAngle(
            Swerve swerveSubsystem,
            Rotation2d targetRotation) {
        this(swerveSubsystem, () -> targetRotation);
    }

    @Override
    public void initialize() {
        this.targetRotation = targetRotationSupplier.get();

        if (targetRotation == null) {
            return;
        }

        Pose2d currentPose = swerveSubsystem.getPose();

        thetaController.setP(Preferences.getDouble(AUTO_ROTATION_P_KEY, SwerveConstants.AUTO_ROTATION_P));

        thetaController.reset(
                currentPose.getRotation().getRadians(),
                swerveSubsystem.getFieldVelocity().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(5.0));
    }

    @Override
    public void execute() {
        if (targetRotation == null) {
            return;
        }

        Pose2d currentPose = swerveSubsystem.getPose();
        Logger.recordOutput("RotateToAngle/current pose", currentPose);
        Logger.recordOutput("RotateToAngle/target rotation", targetRotation);

        thetaErrorAbs =
                Math.abs(
                        currentPose.getRotation().minus(targetRotation).getRadians());

        double ffMinAngle = 0.0;
        double ffMaxAngle = Units.degreesToRadians(10.0);
        double ffScalar =
                MathUtil.clamp(
                        (thetaErrorAbs - ffMinAngle) / (ffMaxAngle - ffMinAngle), 0.0, 1.0);

        double thetaVelocity =
                thetaController.getSetpoint().velocity * ffScalar
                        + thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetRotation.getRadians());

        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        swerveSubsystem.drive(0.0, 0.0, thetaVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return targetRotation == null || thetaController.atGoal();
    }
}
