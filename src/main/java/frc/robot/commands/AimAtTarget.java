package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
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
    Intake intake;
    Supplier<Pose2d> target;
    DoubleSupplier xVel, yVel;
    PIDController rotationPID;
    Timer agitateTimer = new Timer();

    public AimAtTarget(Shooter shooter, Hood hood, Swerve swerve, Kicker kicker, Hopper hopper, Intake intake, Supplier<Pose2d> target, DoubleSupplier xVel, DoubleSupplier yVel) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.target = target;
        this.hood = hood;
        this.kicker = kicker;
        this.hopper = hopper;
        this.intake = intake;
        this.xVel = xVel;
        this.yVel = yVel;
        this.rotationPID = new PIDController(
                SwerveConstants.AUTO_ROTATION_P,
                0.0,
                SwerveConstants.AUTO_ROTATION_D
        );
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Units.degreesToRadians(5.0));

        if (kicker != null && intake != null) {
            addRequirements(swerve, shooter, hood, kicker, intake);
        } else if (kicker != null) {
            addRequirements(swerve, shooter, hood, kicker);
        } else if (intake != null) {
            addRequirements(swerve, shooter, hood, intake);
        } else {
            addRequirements(swerve, shooter, hood);
        }
    }

    public AimAtTarget(Shooter shooter, Hood hood, Swerve swerve, Kicker kicker, Hopper hopper, Supplier<Pose2d> target, DoubleSupplier xVel, DoubleSupplier yVel) {
        this(shooter, hood, swerve, kicker, hopper, null, target, xVel, yVel);
    }

    public AimAtTarget(Shooter shooter, Hood hood, Swerve swerve, Hopper hopper, Supplier<Pose2d> target, DoubleSupplier xVel, DoubleSupplier yVel) {
        this(shooter, hood, swerve, null, hopper, null, target, xVel, yVel);
    }

    @Override
    public void initialize() {
        rotationPID.setP(Preferences.getDouble("Swerve/AutoRotationP", SwerveConstants.AUTO_ROTATION_P));
        rotationPID.setD(Preferences.getDouble("Swerve/AutoRotationD", SwerveConstants.AUTO_ROTATION_D));
        rotationPID.reset();
        agitateTimer.reset();
        agitateTimer.start();
    }

    @Override
    public void execute() {
        double targetAngle = target.get().getTranslation().minus(swerve.getPose().getTranslation()).getAngle().plus(Rotation2d.kPi).getRadians();
        double rotationOutput = rotationPID.calculate(swerve.getPose().getRotation().getRadians(), targetAngle);
        double xParam = MathUtil.applyDeadband(xVel.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED;
        double yParam = MathUtil.applyDeadband(yVel.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED;

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

        boolean shooterReady = shooter.isAtRPM(RPM, 300.0);
        boolean hoodReady = hood.isAtPosition(angle, 0.02);
        boolean swerveReady = rotationPID.atSetpoint();
        boolean driverTranslating = (Math.abs(xParam) > 0.01) || (Math.abs(yParam) > 0.01);

        if (swerveReady && !driverTranslating) {
            swerve.xWheels();
        } else {
            swerve.drive(xParam, yParam, rotationOutput);
        }

        Logger.recordOutput("AimAtTarget/ShooterReady", shooterReady);
        Logger.recordOutput("AimAtTarget/HoodReady", hoodReady);
        Logger.recordOutput("AimAtTarget/SwerveReady", swerveReady);

        boolean targetReady = shooterReady && hoodReady && swerveReady;

        if (kicker != null) {
            double kickerRPM = Preferences.getDouble("Kicker/RPM_SETPOINT", KickerConstants.SHOOTER_RPM);
            if (targetReady) {
                kicker.setRPM(kickerRPM);
            } else {
                kicker.RunWithVoltage(0.0);
            }

            boolean kickerReady = targetReady && kickerRPM > 10.0 && kicker.isAtRPM(kickerRPM, 500.0);
            Logger.recordOutput("AimAtTarget/KickerReady", kickerReady);

            if (kickerReady) {
                if (hopper != null) {
                    hopper.setHopperRightMotorVoltage(6.0);
                    hopper.setLeftHopperMotorVoltage(-6.0);
                }
                if (intake != null) {
                    double amplitude = Preferences.getDouble("AgitateIntake/AmplitudeVolts", 3.0);
                    double frequency = Preferences.getDouble("AgitateIntake/FrequencyHz", 2.0);
                    double agitateVoltage = amplitude * Math.sin(2.0 * Math.PI * frequency * agitateTimer.get());
                    intake.setDeployMotorVoltage(agitateVoltage);
                    Logger.recordOutput("AimAtTarget/AgitateVoltage", agitateVoltage);
                }
            } else {
                if (hopper != null) {
                    hopper.setHopperRightMotorVoltage(0.0);
                    hopper.setLeftHopperMotorVoltage(0.0);
                }
                if (intake != null) {
                    intake.setDeployMotorVoltage(0.0);
                }
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
        if (hopper != null) {
            hopper.setHopperRightMotorVoltage(0.0);
            hopper.setLeftHopperMotorVoltage(0.0);
        }
        if (intake != null) {
            intake.setDeployMotorVoltage(0.0);
        }
        agitateTimer.stop();
    }

}
