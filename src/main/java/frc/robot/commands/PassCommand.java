package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class PassCommand extends Command {

    Shooter shooter;
    Kicker kicker;
    Hood hood;
    Hopper hopper;

    public PassCommand(Shooter shooter, Kicker kicker, Hood hood, Hopper hopper) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.hood = hood;
        this.hopper = hopper;
        addRequirements(shooter, kicker, hood);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.setMotorRPM(3500);
        hood.setHoodPosition(0.4);

        boolean shooterReady = shooter.isAtRPM(3500, 100.0);
        boolean hoodReady = hood.isAtPosition(0.4, 0.02);

        double kickerRPM = Preferences.getDouble("Kicker/RPM_SETPOINT", KickerConstants.SHOOTER_RPM);
        if (kicker.isAtRPM(kickerRPM, 1500.0)) {
            hopper.setHopperRightMotorVoltage(6);
            hopper.setLeftHopperMotorVoltage(-6);
        }
        if (shooterReady && hoodReady) {
            kicker.setRPM(kickerRPM);
        } else {
            kicker.RunWithVoltage(0.0);
            hopper.setHopperRightMotorVoltage(0);
            hopper.setLeftHopperMotorVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotors();
        hood.stopHood();
        kicker.RunWithVoltage(0.0);
        hopper.setHopperRightMotorVoltage(0);
        hopper.setLeftHopperMotorVoltage(0);
    }

}
