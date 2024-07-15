package frc.robot.commands.ArmCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.controller.PIDController;

public class ArmDrivePIDCmd extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
  private double spRest, spCatch, spPoint, spCarry, sp;
  private static final double kP = 9;
  private static final double kI = 0.40;
  private static final double kD = 0.5;
  private final int POV;
  boolean releaseAtSetPoint;

  private PIDController pidController = new PIDController(kP, kI, kD);

  public ArmDrivePIDCmd(ArmSubsystem subsystem, int POV, boolean releaseAtSetPoint){
    this.armSubsystem = subsystem;
    this.spRest = 0.385;
    this.spCatch = 0.325;
    this.spCarry = 0.280;
    this.spPoint = 0.175;
    this.sp = this.spRest;
    this.POV = POV;
    this.pidController.setTolerance(0.01);
    this.releaseAtSetPoint = releaseAtSetPoint;

    addRequirements(subsystem);
  }
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (POV){
      case 0:
        sp = spRest;
        break;
      case 90:
        sp = spCatch;
        break;
      case 180:
        sp = spPoint; 
        break;
      case 270:
        sp = spCarry;
        break;
      }
      pidController.setSetpoint(sp);
      double speed = pidController.calculate(armSubsystem.getAbsEncoder().getAbsolutePosition());
      armSubsystem.armDrive(-speed);
    }
    
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint() && releaseAtSetPoint;
  }

  
  
    
}