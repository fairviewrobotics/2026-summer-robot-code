package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
//import frc.robot.utils.MathUtils;
import frc.robot.utils.MathUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;


public class DriveToPoint extends Command {

    private ProfiledPIDController driveController;
    private ProfiledPIDController thetaController;

    private Swerve swerve;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.0, ffMaxRadius = 0.1;
    private Pose2d targetLocation;
    private Supplier<Pose2d> targetLocationSupplier;

    private static final String AUTO_ROTATION_P_KEY = "DriveToPoint/AutoRotationP";
    private static final String DECELERATION_P_KEY = "DriveToPoint/DecelerationP";

    public DriveToPoint(
            Swerve swerveSubsystem,
            Supplier<Pose2d> targetLocationSupplier,
            double constraintFactor) {
        this.swerve = swerveSubsystem;
        this.targetLocationSupplier = targetLocationSupplier;

        Preferences.initDouble(AUTO_ROTATION_P_KEY, SwerveConstants.DRIVE_P);
        Preferences.initDouble(DECELERATION_P_KEY, SwerveConstants.DRIVE_D);

        this.driveController =
                new ProfiledPIDController(
                        Preferences.getDouble(DECELERATION_P_KEY,SwerveConstants.DRIVE_D),
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                SwerveConstants.MAX_SPEED * constraintFactor,
                                SwerveConstants.MAX_ANGULAR_SPEED * 2.0 * constraintFactor),
                        0.02);

        this.thetaController =
                new ProfiledPIDController(
                        Preferences.getDouble(AUTO_ROTATION_P_KEY, SwerveConstants.AUTO_ROTATION_D),
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                SwerveConstants.MAX_ANGULAR_SPEED,
                                SwerveConstants.MAX_ANGULAR_SPEED * 2.0),
                        0.02);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveSubsystem);
    }

    public DriveToPoint(
            Swerve swerve,
            Pose2d targetLocation,
            double constraintFactor) {
        this(swerve, () -> targetLocation, constraintFactor);
    }

    @Override
    public void initialize() {
        this.targetLocation = targetLocationSupplier.get();

        if (targetLocation == null) {
            return;
        }

        Pose2d currentPose = swerve.getPose();

        driveController.setP(Preferences.getDouble(DECELERATION_P_KEY, SwerveConstants.DRIVE_P));
        thetaController.setP(Preferences.getDouble(AUTO_ROTATION_P_KEY, SwerveConstants.AUTO_ROTATION_P));

        driveController.reset(
                currentPose.getTranslation().getDistance(targetLocation.getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(
                                swerve.getFieldVelocity().vxMetersPerSecond,
                                swerve.getFieldVelocity().vyMetersPerSecond)
                                .rotateBy(
                                        targetLocation
                                                .getTranslation()
                                                .minus(
                                                        swerve
                                                                .getPose()
                                                                .getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                currentPose.getRotation().getRadians(),
                swerve.getFieldVelocity().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(2.0));

        driveController.setTolerance(0.02);
    }

    @Override
    public void execute() {

        if (targetLocation == null) {
            return;
        }

        Pose2d currentPose = swerve.getPose();
        Logger.recordOutput("DriveToPose/current pose", currentPose);
        Logger.recordOutput("DriveToPose/target location", targetLocation);

        double currentDistance =
                currentPose.getTranslation().getDistance(targetLocation.getTranslation());
        double ffScalar =
                MathUtil.clamp(
                        (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveErrorAbs = currentDistance;
        Logger.recordOutput("DriveToPose/ffScalar", ffScalar);
        double driveVelocityScalar =
                driveController.getSetpoint().velocity * ffScalar
                        + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

        double thetaVelocity =
                thetaController.getSetpoint().velocity * ffScalar
                        + thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetLocation.getRotation().getRadians());
        thetaErrorAbs =
                Math.abs(
                        currentPose.getRotation().minus(targetLocation.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        var translationMag = currentPose.getTranslation().minus(targetLocation.getTranslation());

        var driveVelocity =
                MathUtils.getPoseFromRotation(
                                currentPose
                                        .getTranslation()
                                        .minus(targetLocation.getTranslation())
                                        .getAngle())
                        .transformBy(
                                MathUtils.getTransform2dFromTranslation(
                                        new Translation2d(driveVelocityScalar, 0.0)))
                        .getTranslation();
        Translation2d driveVals = new Translation2d(driveVelocity.getX(), driveVelocity.getY());
        swerve.drive(driveVals.getX(), driveVals.getY(), thetaVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return targetLocation == null
                || (driveController.atGoal() && thetaController.atGoal());
    }

}