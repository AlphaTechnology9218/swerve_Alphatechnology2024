package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;


public class ArmTrapezoidSubsystem extends TrapezoidProfileSubsystem{

    CANSparkMax armLeader = new CANSparkMax(ArmConstants.arm0ID, MotorType.kBrushless);
    CANSparkMax armFollower = new CANSparkMax(ArmConstants.arm1ID, MotorType.kBrushless);

    ArmFeedforward armFF = new ArmFeedforward(ArmConstants.Ks, ArmConstants.Kg,
    ArmConstants.Kvspeed, ArmConstants.Kvacce);
    private final RelativeEncoder encoder;
    private final SparkPIDController armPid;

    
    public ArmTrapezoidSubsystem(){
        super(
            new TrapezoidProfile.Constraints(ArmConstants.KMaxVspeedDegreesPerSec,
             ArmConstants.KMaxVacceDegreesPerSecSquared),
            ArmConstants.kArmItialPositionDegrees);

        stop();

        armLeader.restoreFactoryDefaults();
        armFollower.restoreFactoryDefaults();

        armLeader.setIdleMode(IdleMode.kCoast);
        armFollower.setIdleMode(IdleMode.kCoast);

        armLeader.setSmartCurrentLimit(40, 60);
        armFollower.setSmartCurrentLimit(40, 60);

        armLeader.setInverted(true);


        armFollower.follow(armLeader, true);

        encoder = armLeader.getEncoder();
        encoder.setPositionConversionFactor(ArmConstants.ArmConversionFactor);
        encoder.setVelocityConversionFactor(ArmConstants.ArmConversionFactor);

        armPid = armLeader.getPIDController();
        armPid.setP(ArmConstants.kP,0);
        armPid.setI(ArmConstants.Ki,0);
        armPid.setD(ArmConstants.Kd,0);
        armPid.setIZone(ArmConstants.Kiz, 0);
        armPid.setOutputRange(ArmConstants.kArmMinOutput
        ,ArmConstants.kArmMaxOutput);

        armLeader.burnFlash();
        armFollower.burnFlash();
    }

    @Override
    protected void useState(TrapezoidProfile.State setpoint) {
        double feedfoward = armFF.calculate(setpoint.position, setpoint.velocity); 
        armPid.setReference(Units.degreesToRotations(setpoint.position),
        CANSparkBase.ControlType.kPosition, 0, feedfoward);

    }

    @Override
    public void periodic() {
        super.periodic();
        double pos = encoder.getPosition();
        SmartDashboard.putNumber("EncoderCurrentPos", pos);
    }

    public Command setArmGoalCommand(double kArmOffset){
       return Commands.runOnce(() -> setGoal(kArmOffset), this);
    }

    public void stop(){
        armLeader.stopMotor();
    }

}