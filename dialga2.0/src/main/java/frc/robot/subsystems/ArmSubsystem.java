package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
    private static final int arm0ID = 14;

    private static final int arm1ID  = 15;

    private CANSparkMax arm0 = 
    new CANSparkMax(arm0ID, MotorType.kBrushless);

    private CANSparkMax arm1 = 
    new CANSparkMax(arm1ID, MotorType.kBrushless);

    private DutyCycleEncoder absoluteEncoder = 
    new DutyCycleEncoder(5);

    public void armDrive(double val){
        arm0.setOpenLoopRampRate(0.5);
        arm1.setOpenLoopRampRate(0.5);
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
        absoluteEncoder.setPositionOffset(0.2);
        return absoluteEncoder;
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("absoluteArmPosition", getAbsEncoder().getAbsolutePosition());
    }

    
}
