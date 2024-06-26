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

public class AbsoluteDrive extends Command{

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingHorizontal, headingVertical;
    private boolean initRotation = false;

    /*
     * Este comando é utilizado para mover o swerve com uma referência completa ao campo
     * utilizando os inputs de aceleração vX e vY onde x é equivalente para quando o robô 
     * se aproxima/afasta da parede da aliança e y é para esquerda/direita da parede da aliança
     *  e headingHorizontal e headingVertical são coordenada cartesianas de onde o angulo do robô
     * será adquirido, as coordenada serão convertidas para um ângulo polar para onde o robô irá 
     * rotacionar.
     */

     public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
     DoubleSupplier headingVertical)
    {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
        initRotation = true;
    }

    @Override
    public void execute()
    {
        //consegue as Chassis speeds necessárias para um modulo de 2 joysticks
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(),vY.getAsDouble(),
                                                            headingHorizontal.getAsDouble(),
                                                            headingVertical.getAsDouble());

        //impedir movimento após auto
        if(initRotation)
        {
            if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0)
            {
                //pegar orientação atual
                Rotation2d firstloopheading = swerve.getHeading();

                //converte orientação atual para a orientação necessária
                desiredSpeeds = swerve.getTargetSpeeds(
                    0, 0, firstloopheading.getSin(), firstloopheading.getCos());
            }
            //Não inicie a rotação novamente
            initRotation = false;
        }

        //limitar velocidade para evitar a possibilidade do robô tombar
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity()
                                            , swerve.getPose(), Constants.LOOP_TIME, Constants.ROBOT_MASS,
                                            Constants.MATTERLIST, swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("Limited Translation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        //Fazer o robô se mover
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    
    }

    @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished(){
    return false;
  }
    
    

}