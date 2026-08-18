package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.Kicker;

public class KickerCommand extends Command {
    private final Kicker kicker;
    private final double volatage;

    public KickerCommand(Kicker kicker, double volatage){
        this.kicker=kicker;
        this.volatage = volatage;
    }

    @Override
    public void execute(){
        kicker.RunWithVoltage(volatage);
    }

    @Override
    public void end(boolean interrupted){
        kicker.RunWithVoltage(0.0);
    }


}
