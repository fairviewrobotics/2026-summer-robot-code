package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class RefreshPreferences extends InstantCommand {

    private final Swerve swerve;
    private final Intake intake;
    private final Shooter shooter;
    private final Hood hood;
    private final Kicker kicker;

    public RefreshPreferences(Swerve swerve, Intake intake, Shooter shooter, Hood hood, Kicker kicker) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.hood = hood;
        this.kicker = kicker;
    }

    @Override
    public void initialize() {
        swerve.updatePreferences();
        intake.refreshPreferences();
        shooter.refreshPreferences();
        hood.updatePreferences();
        kicker.updatePreferences();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true; // Allows refreshing preferences while disabled in the pits
    }
}