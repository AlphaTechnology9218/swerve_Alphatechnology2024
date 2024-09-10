package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;


import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class FlyWheel extends SubsystemBase{

    private final int flyWheelMotorID0 = 16;
    private final int flyWheelMotorID1 = 17;
    CANSparkMax flyWheelMotor0 = new CANSparkMax(flyWheelMotorID0, MotorType.kBrushless);
    CANSparkMax flyWheelMotor1 = new CANSparkMax(flyWheelMotorID1, MotorType.kBrushless);
    
    //Button xbButton

    public void FlyWheelActive(double speed){
        flyWheelMotor0.setOpenLoopRampRate(1);
        flyWheelMotor1.setOpenLoopRampRate(1);
        flyWheelMotor1.setInverted(false);
        flyWheelMotor0.setInverted(false);
        flyWheelMotor0.set(speed);
        flyWheelMotor1.set(speed);
    }
    public void FlyWheelStop(){
        flyWheelMotor0.set(0);
        flyWheelMotor1.set(0);
    }

    public double FlyWheelSpeedLower(){
        return flyWheelMotor0.getEncoder().getVelocity();
    }

    public double FlyWheelSpeedUpper(){
        return flyWheelMotor1.getEncoder().getVelocity();
    }
   
    }


