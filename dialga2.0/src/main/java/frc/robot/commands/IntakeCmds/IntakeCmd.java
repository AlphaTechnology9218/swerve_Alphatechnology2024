package frc.robot.commands.IntakeCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{
    
    Intake intakeSubsystem;
    boolean sensoValue;
    double sensorNVal;
    Supplier<Double> Trigger0;
    Supplier<Double> Trigger1;

    public IntakeCmd(Intake subsystem, Supplier<Double> Trigger0, Supplier<Double> Trigger1){
        this.intakeSubsystem = subsystem;
        this.Trigger0 = Trigger0;
        this.Trigger1 = Trigger1;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.IntakeDrive((-Trigger0.get() * 0.8+ Trigger1.get()) * 0.8);

        sensoValue = intakeSubsystem.gamePieceIn();

        sensorNVal = intakeSubsystem.getIntakeSensorVal();

        SmartDashboard.putBoolean("Sensor Value Test", sensoValue);
        SmartDashboard.putNumber("Numerical Sensor Value Test", sensorNVal);
        
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
