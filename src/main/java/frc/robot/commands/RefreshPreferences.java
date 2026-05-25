package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class RefreshPreferences extends Command {

    private final Swerve swerve;

    public RefreshPreferences(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.updatePreferences();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}