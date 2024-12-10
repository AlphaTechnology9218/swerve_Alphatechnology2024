package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;


public class FlyWheel extends SubsystemBase{

    CANSparkMax flyWheelLeader = new CANSparkMax(FlywheelConstants.flyWheelMotorID0, MotorType.kBrushless);
    CANSparkMax flyWheelFollower = new CANSparkMax(FlywheelConstants.flyWheelMotorID1, MotorType.kBrushless);
    private final SparkPIDController pid;
    private final RelativeEncoder encoder;
    private  double setpoint;

    public FlyWheel(){
        FlyWheelStop();

        flyWheelLeader.restoreFactoryDefaults();
        flyWheelFollower.restoreFactoryDefaults();

        flyWheelLeader.setIdleMode(IdleMode.kCoast);
        flyWheelFollower.setIdleMode(IdleMode.kCoast);

        flyWheelLeader.setSmartCurrentLimit(40, 60);
        flyWheelFollower.setSmartCurrentLimit(40, 60);

        flyWheelFollower.setInverted(true);

        flyWheelFollower.follow(flyWheelLeader);

        encoder = flyWheelLeader.getEncoder();
        encoder.setPositionConversionFactor(FlywheelConstants.flyWheelConversionFactor);
        encoder.setVelocityConversionFactor(FlywheelConstants.flyWheelConversionFactor);

        pid = flyWheelLeader.getPIDController();
        pid.setP(FlywheelConstants.Kp,0);
        pid.setI(FlywheelConstants.Ki,0);
        pid.setD(FlywheelConstants.Kd,0);  
        pid.setFF(1/473,0);

        flyWheelLeader.burnFlash();
        flyWheelFollower.burnFlash();
    } 

    public void FlyWheelActive(double setpointRPM){
        pid.setReference(setpointRPM
        , ControlType.kVelocity);
        setpoint = setpointRPM;
    }

    

    public void FlyWheelStop(){
        flyWheelLeader.stopMotor();
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FlyWheelMotorVelocity", encoder.getVelocity());
        SmartDashboard.putNumber("FlyWheelSetPoint", setpoint);   
    }

    }


