package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.KickerConstants;

public class Kicker {
    TalonFX kickerMotor1 = new TalonFX(KickerConstants.KICKER_MOTOR_ID_1);
    TalonFX kickerMotor2 = new TalonFX(KickerConstants.KICKER_MOTOR_ID_2);

    public void RunWithVoltage(double voltage){
        kickerMotor1.setVoltage(voltage);
        kickerMotor2.setVoltage(voltage);
    }

}
