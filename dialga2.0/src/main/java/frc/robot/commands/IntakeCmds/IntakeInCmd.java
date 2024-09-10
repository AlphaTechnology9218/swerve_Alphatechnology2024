package frc.robot.commands.IntakeCmds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeInCmd extends Command{
    Intake intakeSubsystem;

    public IntakeInCmd(Intake subsystem){
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intakeSubsystem.IntakeDrive(-1);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.IntakeStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}