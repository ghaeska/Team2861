// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.SwerveConstants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/* Subsystem Imports */

/* Pathplanner Imports */

/* Smartdashboard Imports */



/*
** This class is where the bulk of the robot should be declared. Since Command-based is a
** "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
** periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
** subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer 
{
  /* Subsytem initilization */
  // TODO: Add Robot subsystems here.  Arms, elevators, vision systems.... also our auto chooser.


  /* The controller that are used to control the robot.  Initialized here. */
  // TODO: Make two xbox controllers.  One for Driver and one for operator.
  // Note: Instead of "OperatorConstants.kDriverControllerPort" we use the following "OIConstants.kDriverControllerPort"
  // Because we have them defined in the OIConstants structure, not the operator structure.
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /* Named Commands used in Auto */
  private void registerNamedCommands()
  {
    //This is blank for now, until we make commands to be used with pathplanner.
  }

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    /* Call the command to get the registered commands. */
    registerNamedCommands();

    /* Create an Auto Selector */
    // Once we have more code added and start doing more Auto Routines, we will need to add a AutoChooser.

    /* Configure the button bindings */ 
    configureButtonBindings();

    /* Add Default Commands here. */
    // TODO: We should need to add the swerve drive default here.
  }

  /* Configure all the buttons for the controller. */
  private void configureButtonBindings() 
  {
    
  }

  /* Send Commands to the Auto Chooser.  */
  // TODO: Uncomment this when we have commands to send.
  // public Command getAutonomousCommand() 
  // {
  //   
  // }
}
