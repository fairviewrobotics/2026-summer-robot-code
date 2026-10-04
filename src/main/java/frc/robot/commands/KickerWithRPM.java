package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.Kicker;

public class KickerWithRPM extends Command {
    private final Kicker kicker;
    private final double RPM;

    public KickerWithRPM(Kicker kicker){
        this.kicker=kicker;
        this.RPM = Preferences.getDouble("Kicker/RPM_SETPOINT", 0);
    }

    @Override
    public void execute(){
        kicker.setRPM(RPM);
    }

    @Override
    public void end(boolean interrupted){
        kicker.RunWithVoltage(0.0);
    }


}