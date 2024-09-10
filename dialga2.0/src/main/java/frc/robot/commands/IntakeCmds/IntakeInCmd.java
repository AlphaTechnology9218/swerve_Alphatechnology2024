package frc.robot.commands.IntakeCmds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeInCmd extends Command{
    Intake intakeSubsystem;
    boolean in;
    public IntakeInCmd(Intake subsystem){
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        in = intakeSubsystem.gamePieceIn();
        intakeSubsystem.IntakeDrive(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.IntakeStop();
    }

    @Override
    public boolean isFinished() {
        boolean finished;
        if(in == true){
            finished = true;
        }else{
            finished = false;
        }
        return finished;
    }
}