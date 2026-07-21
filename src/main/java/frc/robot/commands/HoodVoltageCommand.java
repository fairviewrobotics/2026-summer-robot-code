package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class HoodVoltageCommand extends Command {

    private final Hood hood;
    private double voltage;

    public HoodVoltageCommand(Hood hood, double voltage) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() { hood.setVoltage(voltage); }

    @Override
    public void end(boolean interrupted) { hood.setVoltage(0); }
}
