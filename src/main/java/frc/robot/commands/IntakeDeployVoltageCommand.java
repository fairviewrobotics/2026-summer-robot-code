package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDeployVoltageCommand extends Command {
    private final Intake intake;
    private double voltage;

    public IntakeDeployVoltageCommand(Intake intake, double voltage) {
        this.intake = intake;
        this.voltage = voltage;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setDeployMotorVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) { intake.setDeployMotorVoltage(0); }
}
