package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;

public class RefreshPreferences extends Command {

    Swerve swerve;

    public RefreshPreferences(Swerve swerve) {
        swerve.updatePreferences();
    }

}
