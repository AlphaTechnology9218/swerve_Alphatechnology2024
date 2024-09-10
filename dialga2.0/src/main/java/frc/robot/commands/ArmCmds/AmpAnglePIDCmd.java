package frc.robot.commands.ArmCmds;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AmpAnglePIDCmd extends Command{
    ArmSubsystem armSubsystem;
    private static final double kP = 5.5;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double sp = 0.655;


    private PIDController pidController = new PIDController(kP, kI, kD);
    

    public AmpAnglePIDCmd(ArmSubsystem subsystem){
        this.armSubsystem = subsystem;
        pidController.setTolerance(0.1);
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      pidController.setSetpoint(sp);
    //&& armSubsystem.getAbsEncoder().getAbsolutePosition() >= sp
      double speed = pidController.calculate(armSubsystem.getAbsEncoder().getAbsolutePosition());
      if (armSubsystem.getAbsEncoder().getAbsolutePosition() != 0){
        if (speed < 0 ){
          armSubsystem.armDrive(0.2);
        }
        else{
          armSubsystem.armDrive(-speed);
        }
      }
      
    }
    
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
