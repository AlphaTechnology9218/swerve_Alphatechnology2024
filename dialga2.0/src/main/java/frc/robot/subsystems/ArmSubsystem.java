/*package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {


    CANSparkMax arm0 = 
    new ConfiguredCanSparkMax().BrushlessMotor(ArmConstants.arm0ID, IdleMode.kBrake);
    CANSparkMax arm1 = 
    new ConfiguredCanSparkMax().BrushlessMotor(ArmConstants.arm1ID, IdleMode.kBrake);

    private DutyCycleEncoder absoluteEncoder = 
    new DutyCycleEncoder(ArmConstants.armAbsIncoderChannel);

    public void armDrive(double val){
        arm0.setOpenLoopRampRate(ArmConstants.armDriveOpenLoopRate);
        arm1.setOpenLoopRampRate(ArmConstants.armDriveOpenLoopRate);
        arm0.setInverted(false);
        arm1.setInverted(true); 
        arm0.set(val);
        arm1.set(val);
    } 

    public void stopArm(){
        arm0.set(0);
        arm1.set(0);
    }

    public DutyCycleEncoder getAbsEncoder(){
        absoluteEncoder.setPositionOffset(ArmConstants.armABSEEncoderOffset);
        return absoluteEncoder;   
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("absoluteArmPosition", getAbsEncoder().getAbsolutePosition() - getAbsEncoder().getPositionOffset());
    }

    
}*/
