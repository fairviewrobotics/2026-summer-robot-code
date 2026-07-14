package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
    private final Shooter shooterSubsystem;

    public ShooterCommand(Shooter shooterSubsystem, double RPM) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        double RPM = Preferences.getDouble("Shooter/RPM_SETPOINT", ShootingConstants.SHOOTER_RPM);
        double shooterSetpoint = RPM * Preferences.getDouble("AimAtHub/SHOOTER_RPM_SCALAR", 1.0);
        shooterSubsystem.setMotorRPM(shooterSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopMotors();
    }

}