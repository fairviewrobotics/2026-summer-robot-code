package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class HoodPreferencesCommand extends Command {
    private final Hood hood;

    public HoodPreferencesCommand(Hood hood) {
        this.hood = hood;

        addRequirements(hood);
    }

    // TODO: Change backup value
    @Override
    public void execute() { hood.setHoodPosition(Preferences.getDouble("Hood/TARGET_ANGLE", 0.0)); }

    @Override
    public void end(boolean interrupted) { hood.stopHood(); }
}
