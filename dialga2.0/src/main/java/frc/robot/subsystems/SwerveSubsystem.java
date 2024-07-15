package frc.robot.subsystems;
import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  
   
    private final SwerveDrive swerveDrive;

    public double maximumSpeed = Units.feetToMeters(14.5);

    public SwerveSubsystem(File directory){
        //Este valor é utilizado para converter as rotações do motor para as rotações que a roda faz em seu próprio eixo, 
        //ele é calculado utilizando a equação 360/(RELAÇÃO DE ENGRENAGENS DA ROTAÇÃO * RESOLUÇÃO DO ENCODER)
        //Ou Utilizando a função abaixo
        double angleConversionFactor = 
        SwerveMath.calculateDegreesPerSteeringRotation(Constants.SwerveConstants.kSwerveSteeringRatio);


         //Já este Valor é o valor de conversão das rotações do motor para a movimentação das rodas
         //Ele é calculado da seguinte maneira (PI * DIÂMETRO DA RODA EM METROS)/(RELAÇÃO DE ENGRENAGENS DE ACELERAÇÃO * RESOLUÇÃO DO ENCODER)
        double driveConvarsionFactor = 
        SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);

        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConvarsionFactor);
        System.out.println("}");

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try{
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        }catch (Exception e){
            throw new RuntimeException(e);   
        }
        
        
        setupPathPlanner();
    }

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controlCfg)
    {
        swerveDrive = new SwerveDrive(driveCfg, controlCfg, maximumSpeed);
    }
    /*
     * Setup AutoBuilder for PathPlanner
     */
    public void setupPathPlanner(){
        AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry, 
        this::getRobotVelocity, 
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
                                        Constants.AutonConstants.TRANSLATION_PID, 
                                        Constants.AutonConstants.ANGLE_PID,
                                         4.5,
                                         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         new ReplanningConfig()
        
        ), 
        () -> {
            // supplier booleana que inverte o caminho pro lado vermelho da aliança
            // isso vai inverter o caminho pro lado vermelho do campo.
            // A ORIGEM CONTINUARÁ NO LADO AZUL
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
          },
          this //referência a este subsistema para definir parâmetros
                                    );
    }

    public Command getAutonomousCommand(String Pathname){
    //Cria um comando seguidor de trajetoria usando o autoBuilder
    return new PathPlannerAuto(Pathname); 
    }

    public Command driveToPose(Pose2d pose){
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumVelocity(),4.0,
            swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720)
            );

            return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0,
                0.0   
            );
    }
    //Comando de controle remoto do swerve utilizando a orientação e os translation values como setpoint
    //@param locomoção x e y das rodas elevado a 3 para controles mais fluidos
    //@param orientação x e y do robô para calcular o ângulo dos joysticks

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
    DoubleSupplier headingX, DoubleSupplier headingY)
    {
        swerveDrive.setHeadingCorrection(true);
        return run(()-> {
            double xInput = Math.pow(translationX.getAsDouble(), 3);
            double yInput = Math.pow(translationY.getAsDouble(), 3);
            //faz o robô se mover
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput,yInput,
                                                                            translationX.getAsDouble(),
                                                                            translationY.getAsDouble(),
                                                                            swerveDrive.getOdometryHeading().getRadians(),
                                                                            swerveDrive.getMaximumVelocity()));
        });
    }

    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation){
        //swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
        // Make the robot move
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                        translationY.getAsDouble(),
                                                                        rotation.getAsDouble() * Math.PI,
                                                                        swerveDrive.getOdometryHeading().getRadians(),
                                                                        swerveDrive.getMaximumVelocity()));
      });
    }
    //Comando de caracterização dos motores de locomoção pelo sysID
        public Command sysIdDriveMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12),
        3.0, 5.0, 3.0);
     }
     //Comando de caracterização dos motores de ângulo pelo sysID
        public Command sysIdAngleMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                new Config(),
                this, swerveDrive),
            3.0, 5.0, 3.0);
    }

    //Comando de locomoção remota utilizando os translation values e orientação como angular velocity
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
     DoubleSupplier angularRotationX)
  {
    swerveDrive.setHeadingCorrection(true);
    swerveDrive.setCosineCompensator(true);
    swerveDrive.setChassisDiscretization(true, 0.005);
    return run(() -> {
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  /*@param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   *
   *@param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   */
    //Movimentação sem utilizar o campo como referência
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                        rotation,
                        fieldRelative,
                        false); // Open loop is disabled since it shouldn't be used most of the time.
    }

     public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void drive(ChassisSpeeds velocity)
    {
        swerveDrive.drive(velocity);
    }
    
    @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance()
  {
    if(isRedAlliance() == true)
    {
      zeroGyro();

      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    }else{
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3,3,Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }




}






