package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterVoltageCommand extends Command {
    private final Shooter shooterSubsystem;
    private double voltage;

    public ShooterVoltageCommand(Shooter shooterSubsystem, double voltage) {
        this.shooterSubsystem = shooterSubsystem;
        this.voltage = voltage;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setLeftShooterMotorVoltage(voltage);
        shooterSubsystem.setRightShooterMotorVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setLeftShooterMotorVoltage(0);
        shooterSubsystem.setRightShooterMotorVoltage(0);
    }
}
