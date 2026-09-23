package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RefreshPreferences extends InstantCommand {

    private final Swerve swerve;
    private final Intake intake;
    private final Shooter shooter;
    private final Hood hood;

    public RefreshPreferences(Swerve swerve, Intake intake, Shooter shooter, Hood hood) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.hood = hood;
        // Do NOT add requirements so it won't interrupt active driving or intake commands
    }

    @Override
    public void initialize() {
        swerve.updatePreferences();
        intake.refreshPreferences();
        shooter.updateHardwareConfigs();
        hood.updatePreferences();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true; // Allows refreshing preferences while disabled in the pits
    }
}