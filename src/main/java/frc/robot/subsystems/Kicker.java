package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.KickerConstants;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    TalonFX kickerMotor1 = new TalonFX(KickerConstants.KICKER_MOTOR_ID_1);
    TalonFX kickerMotor2 = new TalonFX(KickerConstants.KICKER_MOTOR_ID_2);
    TalonFXConfiguration kickerMotor1Config = new TalonFXConfiguration();
    TalonFXConfiguration kickerMotor2Config = new TalonFXConfiguration();
    private PIDController KickerPID = new PIDController(KickerConstants.DEFAULT_KP, KickerConstants.DEFAULT_KI, KickerConstants.DEFAULT_KD);
    private SimpleMotorFeedforward KickerFF = new SimpleMotorFeedforward(KickerConstants.DEFAULT_KS, KickerConstants.DEFAULT_KV);


    public Kicker() {
        kickerMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kickerMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        initializePreferences();
    }

    private void initializePreferences(){
        Preferences.initDouble("Kicker/kP", KickerConstants.DEFAULT_KP);
        Preferences.initDouble("Kicker/kI", KickerConstants.DEFAULT_KI);
        Preferences.initDouble("Kicker/kD", KickerConstants.DEFAULT_KD);
        Preferences.initDouble("Kicker/kV", KickerConstants.DEFAULT_KV);
        Preferences.initDouble("Kicker/kS", KickerConstants.DEFAULT_KS);
        Preferences.initDouble("Kicker/RPM_SETPOINT", KickerConstants.SHOOTER_RPM);
    }

    public void updatePreferences() {
        KickerPID.setP(Preferences.getDouble("Kicker/kP", KickerConstants.DEFAULT_KP));
        KickerPID.setI(Preferences.getDouble("Kicker/kI", KickerConstants.DEFAULT_KI));
        KickerPID.setD(Preferences.getDouble("Kicker/kD", KickerConstants.DEFAULT_KD));
        KickerFF.setKv(Preferences.getDouble("Kicker/kV", KickerConstants.DEFAULT_KV));
        KickerFF.setKs(Preferences.getDouble("Kicker/kS", KickerConstants.DEFAULT_KS));

    }


    public void RunWithVoltage(double voltage){
        kickerMotor1.setVoltage(-voltage);
        kickerMotor2.setVoltage(voltage);
    }

    public void setRPM(double RPM){
        double PIDCalc = KickerPID.calculate(kickerMotor1.getVelocity().getValueAsDouble() * 60, RPM);
        double FFcalc = KickerFF.calculate(RPM);
        RunWithVoltage(-(PIDCalc + FFcalc));
    }
    @Override
    public void periodic(){
        Logger.recordOutput("Kicker/CurrentRPM", kickerMotor1.getVelocity().getValueAsDouble()*60);
    }


}