package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;


public class ArmTrapezoidSubsystem extends TrapezoidProfileSubsystem{

    CANSparkMax armLeader = new CANSparkMax(ArmConstants.arm0ID, MotorType.kBrushless);
    CANSparkMax armFollower = new CANSparkMax(ArmConstants.arm1ID, MotorType.kBrushless);
    DigitalInput botLimitSwitch = new DigitalInput(7);  

    ArmFeedforward armFF = new ArmFeedforward(ArmConstants.Ks, ArmConstants.Kg,
    ArmConstants.Kvspeed, ArmConstants.Kvacce);
    private final RelativeEncoder encoder;
    private final SparkPIDController armPid;

    
    public ArmTrapezoidSubsystem(){
        super(
            new TrapezoidProfile.Constraints(ArmConstants.KMaxVspeedDegreesPerSec,
             ArmConstants.KMaxVacceDegreesPerSecSquared),
            ArmConstants.kArmItialPositionDegrees);

        

        armLeader.restoreFactoryDefaults();
        armFollower.restoreFactoryDefaults();

        armLeader.setIdleMode(IdleMode.kBrake);
        armFollower.setIdleMode(IdleMode.kBrake);

        armLeader.setSmartCurrentLimit(40, 60);
        armFollower.setSmartCurrentLimit(40, 60);

        armLeader.setInverted(true);


        armFollower.follow(armLeader, true);

        encoder = armLeader.getEncoder();
        encoder.setPositionConversionFactor(ArmConstants.ArmPosConversionFactor);
        encoder.setVelocityConversionFactor(ArmConstants.ArmVeloConversionFactor);

        armPid = armLeader.getPIDController();
        armPid.setP(ArmConstants.Kp,0);
        armPid.setI(ArmConstants.Ki,0);
        armPid.setD(ArmConstants.Kd,0);
        armPid.setIZone(ArmConstants.Kiz, 0);

        armLeader.burnFlash();
        armFollower.burnFlash();

        resetEncoder();
    }

    @Override
    protected void useState(TrapezoidProfile.State setpoint) {
        double feedfoward = armFF.calculate(setpoint.position, setpoint.velocity); 
        armPid.setReference(setpoint.position,
        CANSparkBase.ControlType.kPosition, 0, feedfoward);

    }

    @Override
    public void periodic() {
        super.periodic();
        if(botLimitSwitch.get() == true){
            encoder.setPosition(0);
            armPid.setOutputRange(0, ArmConstants.kArmMaxOutput);
        }else if(botLimitSwitch.get() ==  false){
            armPid.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);
        }
        double pos = encoder.getPosition();
        double velo = encoder.getVelocity();
        SmartDashboard.putNumber("EncoderCurrentPos", pos);
        SmartDashboard.putNumber("EncoderCurrentVelo", velo);
        SmartDashboard.putNumber("EncoderVeloSetPoint", 30);
        SmartDashboard.putBoolean("LimitSwitchActive", botLimitSwitch.get()); 
    }

    public Command setArmGoalCommand(double kArmOffset, double kVelocityOffset){
       return Commands.runOnce(
        () -> setGoal(new TrapezoidProfile.State(kArmOffset, kVelocityOffset)), this);
    }

    public void stop(){
        armLeader.stopMotor();
    }

    public void resetEncoder(){
     if(botLimitSwitch.get() == true){
          encoder.setPosition(0);
          armLeader.stopMotor();
     }else if(botLimitSwitch.get() == false){
        armLeader.set(-0.2);
     }
    }
        

}