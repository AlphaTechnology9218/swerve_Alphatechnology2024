package frc.robot.commands.ArmCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDrive extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem arm;
    private Supplier<Double> speed;

    public ArmDrive(ArmSubsystem subsystem, Supplier<Double> speed){
        this.arm = subsystem;
        this.speed = speed;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
       SmartDashboard.
        putNumber("absoluteArmPosition", arm.getAbsEncoder().getAbsolutePosition());
        arm.armDrive(speed.get()); 
    }

    @Override
    public void end(boolean interrupted){
        arm.stopArm();
    }

    @Override
    public boolean isFinished(){
        return false;  
    }
}
