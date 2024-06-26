package frc.robot.commands.driveBase;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteFieldDrive extends Command{

    private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY, heading;

    //Esse é similar ao absolute drive só que sem evitar a rotação após o autonomo
  public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
  DoubleSupplier heading)
    {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        //Pegar as chassisSpeeds desejadas de 2 joysticks
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble()
                                                            , new Rotation2d(heading.getAsDouble() * Math.PI));
        
        //limite de velocidade para evitar tombar

        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(),
         swerve.getPose(), Constants.LOOP_TIME, Constants.ROBOT_MASS, Constants.MATTERLIST,
          swerve.getSwerveDriveConfiguration());

        SmartDashboard.putNumber("Limited Translation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        //Faz o robô se mover
        swerve.drive(translation,desiredSpeeds.omegaRadiansPerSecond, true);
        
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}