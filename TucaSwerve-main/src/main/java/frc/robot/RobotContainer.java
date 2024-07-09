// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    private SendableChooser<Command> chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandJoystick driverFlightStick = new CommandJoystick(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
                                                                         
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
   initializeChooser();
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation

   Command baseDriveCommand = drivebase.driveCommand(        
      () -> MathUtil.applyDeadband(driverFlightStick.getY() * -0.7, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverFlightStick.getX() * -0.7, OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverFlightStick.getRawAxis(2)*-0.7,.1));
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive


    drivebase.setDefaultCommand(
       baseDriveCommand);
  }

  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    driverFlightStick.trigger().onTrue(Commands.runOnce(drivebase::zeroGyro));


    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }
private void initializeChooser(){
chooser.addOption("Taxi Auto", new PathPlannerAuto("New Auto"));
 
SmartDashboard.putData("CHOOSE", chooser);


}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto(){
    return chooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
