package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimAtTarget extends Command {

    Swerve swerve;
    Shooter shooter;
    Hood hood;
    Supplier<Pose2d> target;
    DoubleSupplier xVel, yVel;
    ProfiledPIDController rotationPID;

    public AimAtTarget(Shooter shooter, Hood hood, Swerve swerve, Supplier<Pose2d> target, DoubleSupplier xVel, DoubleSupplier yVel) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.hood = hood;
        this.xVel = xVel;
        this.yVel = yVel;
        this.rotationPID = new ProfiledPIDController(
                SwerveConstants.AUTO_ROTATION_P,
                0.0,
                SwerveConstants.AUTO_ROTATION_D,
                SwerveConstants.AUTO_ROTATION_CONSTRAINTS
        );
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Units.degreesToRadians(2.0));
        addRequirements(swerve, shooter);
    }

    @Override
    public void initialize() {
        rotationPID.setP(Preferences.getDouble("Swerve/AutoRotationP", SwerveConstants.AUTO_ROTATION_P));
        rotationPID.setD(Preferences.getDouble("Swerve/AutoRotationD", SwerveConstants.AUTO_ROTATION_D));
        double maxVelocity = Preferences.getDouble("Swerve/AutoRotationMaxVelocity", SwerveConstants.AUTO_ROTATION_CONSTRAINTS.maxVelocity);
        double maxAcceleration = Preferences.getDouble("Swerve/AutoRotationMaxAcceleration", SwerveConstants.AUTO_ROTATION_CONSTRAINTS.maxAcceleration);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        rotationPID.setConstraints(constraints);
        rotationPID.reset(swerve.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        double targetAngle = target.get().minus(swerve.getPose()).getRotation().plus(Rotation2d.kPi).getRadians();
        double rotationOutput = rotationPID.calculate(swerve.getPose().getRotation().getRadians(), targetAngle);
        double xParam = MathUtil.applyDeadband(xVel.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED;
        double yParam = MathUtil.applyDeadband(yVel.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED;
        swerve.drive(xParam, yParam, rotationOutput);
        double distance = Math.hypot(
                Math.abs(target.get().getX() - swerve.getPose().getX()),
                Math.abs(target.get().getY() - swerve.getPose().getY()));
        double RPM = shooter.getDistanceToRPMM(distance);
        double angle = hood.getDistanceToAngle(distance);
        shooter.setLeftShooterMotor(RPM);
        shooter.setRightShooterMotor(RPM);
        hood.setHoodPosition(angle);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHoodPosition(0);
        shooter.setLeftShooterMotor(2000);
        shooter.setRightShooterMotor(2000);
    }

}
