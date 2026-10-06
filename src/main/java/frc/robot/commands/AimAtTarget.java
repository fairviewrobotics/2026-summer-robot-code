package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.*;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimAtTarget extends Command {

    Swerve swerve;
    Shooter shooter;
    Hood hood;
    Kicker kicker;
    Hopper hopper;
    Supplier<Pose2d> target;
    DoubleSupplier xVel, yVel;
    ProfiledPIDController rotationPID;

    public AimAtTarget(Shooter shooter, Hood hood, Swerve swerve, Kicker kicker, Hopper hopper, Supplier<Pose2d> target, DoubleSupplier xVel, DoubleSupplier yVel) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.target = target;
        this.hood = hood;
        this.kicker = kicker;
        this.hopper = hopper;
        this.xVel = xVel;
        this.yVel = yVel;
        this.rotationPID = new ProfiledPIDController(
                SwerveConstants.AUTO_ROTATION_P,
                0.0,
                SwerveConstants.AUTO_ROTATION_D,
                SwerveConstants.AUTO_ROTATION_CONSTRAINTS
        );
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Units.degreesToRadians(5.0));
        if (kicker != null) {
            addRequirements(swerve, shooter, hood, kicker);
        } else {
            addRequirements(swerve, shooter, hood);
        }
    }

    public AimAtTarget(Shooter shooter, Hood hood, Swerve swerve, Hopper hopper, Supplier<Pose2d> target, DoubleSupplier xVel, DoubleSupplier yVel) {
        this(shooter, hood, swerve, null, hopper, target, xVel, yVel);
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
        double targetAngle = target.get().getTranslation().minus(swerve.getPose().getTranslation()).getAngle().plus(Rotation2d.kPi).getRadians();
        double rotationOutput = rotationPID.calculate(swerve.getPose().getRotation().getRadians(), targetAngle);
        double xParam = MathUtil.applyDeadband(xVel.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED;
        double yParam = MathUtil.applyDeadband(yVel.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED;
        swerve.drive(xParam, yParam, rotationOutput);
        double distance = Math.hypot(
                Math.abs(target.get().getX() - swerve.getPose().getX()),
                Math.abs(target.get().getY() - swerve.getPose().getY()));
        Logger.recordOutput("AimAtTarget/Distance", distance);
        double RPM = shooter.getDistanceToRPM(distance);
        double angle = hood.getDistanceToAngle(distance);
        Logger.recordOutput("AimAtTarget/RPM", RPM);
        Logger.recordOutput("AimAtTarget/Angle", angle);
        shooter.setMotorRPM(RPM);
        hood.setHoodPosition(angle);

        boolean shooterReady = shooter.isAtRPM(RPM, 100.0);
        boolean hoodReady = hood.isAtPosition(angle, 0.02);
        boolean swerveReady = rotationPID.atGoal();

        Logger.recordOutput("AimAtTarget/ShooterReady", shooterReady);
        Logger.recordOutput("AimAtTarget/HoodReady", hoodReady);
        Logger.recordOutput("AimAtTarget/SwerveReady", swerveReady);

        if (kicker != null) {
            double kickerRPM = Preferences.getDouble("Kicker/RPM_SETPOINT", KickerConstants.SHOOTER_RPM);
            if (kicker.isAtRPM(kickerRPM, 1500.0)) {
                hopper.setHopperRightMotorVoltage(6);
                hopper.setLeftHopperMotorVoltage(-6);
            }
            if (shooterReady && hoodReady && swerveReady) {
                kicker.setRPM(kickerRPM);
            } else {
                kicker.RunWithVoltage(0.0);
                hopper.setHopperRightMotorVoltage(0);
                hopper.setLeftHopperMotorVoltage(0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHoodPosition(0);
        shooter.stopMotors();
        if (kicker != null) {
            kicker.RunWithVoltage(0.0);
        }
        hopper.setHopperRightMotorVoltage(0);
        hopper.setLeftHopperMotorVoltage(0);
    }

}
