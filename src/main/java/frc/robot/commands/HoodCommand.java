package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.Hood;

public class HoodCommand extends Command {

    private final Hood hood;
    private double angle;

    public HoodCommand(Hood hood, double angle) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setHoodPosition(angle);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHoodPosition(0);
    }

}
