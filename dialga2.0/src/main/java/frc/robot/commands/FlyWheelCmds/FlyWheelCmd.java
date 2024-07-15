package frc.robot.commands.FlyWheelCmds;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheel;



public class FlyWheelCmd extends Command {
   
    FlyWheel flyWheelSubsystem;
    

   
 

 public FlyWheelCmd(FlyWheel subsystem) {
        this.flyWheelSubsystem = subsystem;
        addRequirements(subsystem);  
 }

    @Override
    public void initialize() {
           
    }
    @Override
    public void execute() {
      flyWheelSubsystem.FlyWheelActive(1);

    }
    @Override
    public void end(boolean interrupted) {
        flyWheelSubsystem.FlyWheelStop();   
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}