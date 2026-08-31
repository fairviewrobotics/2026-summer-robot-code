package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class RefreshPreferences extends Command {

    private final Swerve swerve;
    private final Intake intake;

    public RefreshPreferences(Swerve swerve, Intake intake) {
        this.swerve = swerve;
        this.intake = intake;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.updatePreferences();
        intake.refreshPreferences();
    }

}