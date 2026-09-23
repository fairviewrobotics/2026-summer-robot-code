package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;



import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.ShootingConstants;

public class Hopper extends SubsystemBase {

    private SparkFlex leftHopperMotor;
    private SparkFlex rightHopperMotor;

    public Hopper(){
        leftHopperMotor = new SparkFlex(HopperConstants.HOPPER_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        rightHopperMotor = new SparkFlex(HopperConstants.HOPPER_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    }

    public void setLeftHopperMotorVoltage(double voltage){
        leftHopperMotor.setVoltage(-voltage);
    }

    public void setHopperRightMotorVoltage(double voltage){
        rightHopperMotor.setVoltage(voltage);
    }


}