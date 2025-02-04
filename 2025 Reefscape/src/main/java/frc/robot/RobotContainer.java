// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.OIConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/* Pathplanner Calls */
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/* Smartdashboard Calls */
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // TODO: Elevator subsystem here.
  // TODO: coral manipulator system here.
  // TODO: algea manipulator system here.
  // TODO: climb system manipulator here.

  /* TODO: Auto stuff here.  Line 69-99 in 2024 code. */


  /* The controller that are used to control the robot.  Initialized here. */
  CommandXboxController m_OperatorController = new CommandXboxController( OIConstants.k2ndDriverControllerPort );
  CommandXboxController m_xboxController     = new CommandXboxController( OIConstants.kDriverControllerPort );
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // TODO: add auto chooser stuff here.
    final SendableChooser<Command> autoChooser;// = new SendableChooser<>();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand( () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_OperatorController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_OperatorController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_OperatorController.getRightX(), OIConstants.kDriveDeadband),
        true,
        true),
        m_robotDrive ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    m_xboxController.rightStick().whileTrue( new RunCommand( () -> m_robotDrive.setX(), m_robotDrive ));

    // TODO: Add more button bindings here.
  } 

  private void configureNewCommands()
  {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   // TODO: Update this auto stuff.
  //public Command getAutonomousCommand() { }

} 
