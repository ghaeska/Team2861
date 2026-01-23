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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();

  /* The controller that are used to control the robot.  Initialized here. */
  // TODO, Done: Make two xbox controllers.  One for Driver and one for operator.
  // Note: Instead of "OperatorConstants.kDriverControllerPort" we use the following "OIConstants.kDriverControllerPort"
  // Because we have them defined in the OIConstants structure, not the operator structure.
  CommandXboxController m_DriverController = new CommandXboxController( OIConstants.kDriverControllerPort );
  CommandXboxController m_OperatorController = new CommandXboxController( OIConstants.k2ndDriverControllerPort );

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
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand( () -> m_robotDrive.drive(
        getDriveForward(),
        getDriveStrafe(),
        getDriveRotation(),
        DriveFieldRelative(),
        false),
        m_robotDrive ) );
  }

private boolean DriveFieldRelative()
  {
    if( m_DriverController.leftTrigger().getAsBoolean() == true )
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  private Command DriveStop()
  {
    return new RunCommand( 
      () -> m_robotDrive.drive( 
      0, 
      0,
      0, 
      true, 
      false), 
      m_robotDrive );
  }

  private double getDriveStrafe()
  {
    double controllerStrafe = -MathUtil.applyDeadband(m_DriverController.getLeftX(), OIConstants.kDriveDeadband);
    if( m_DriverController.leftTrigger().getAsBoolean() == true )
    {
      //check to see if the area is large enough to assume we are lined up.
      double m_strafe = m_vision.getLimelightTA();
      if( m_strafe >= 11.5 )
      {
        //we are close to on center, stop allowing strafe movements.
        controllerStrafe = 0;
      }      
    }
    return controllerStrafe;
  }

  private double getDriveForward()
  {
    double controllerForward = -MathUtil.applyDeadband(m_DriverController.getLeftY(), OIConstants.kDriveDeadband);
    if( m_DriverController.leftTrigger().getAsBoolean() == true )
    {
      double m_fwd = m_vision.limelight_range_proportional();
      return m_fwd;
      //return controllerForward;
    }
    else 
    {
      return controllerForward;
    }
  }

  private double getDriveRotation() 
  {
    double controllerAngle = -MathUtil.applyDeadband(m_DriverController.getRightX(), OIConstants.kDriveDeadband) ;
    if( m_DriverController.leftTrigger().getAsBoolean() == true ) 
    {
      Double m_rot = m_vision.limelight_aim_proportional();
      
      return m_rot;
      
    } 
    else 
    {
      return controllerAngle;
    }
  }
  /* Configure all the buttons for the controller. */
  private void configureButtonBindings() 
  {
     /************************* DriveTrain Commands ****************************/
    /* Command to set wheels in X formation when right stick gets pushed down. */
    m_DriverController.rightStick().whileTrue( new RunCommand( () -> m_robotDrive.setX(), m_robotDrive ));

    /* Command to reset the robot heading when "start" gets pushed. */
    m_DriverController.start().onTrue(new InstantCommand( m_robotDrive::zeroHeading ).ignoringDisable(true) );
    m_OperatorController.start().onTrue( new InstantCommand( m_robotDrive::zeroHeading ).ignoringDisable(true) );

  }

  /* Send Commands to the Auto Chooser.  */
  // TODO: Uncomment this when we have commands to send.
  // public Command getAutonomousCommand() 
  // {
  //   
  // }
}
