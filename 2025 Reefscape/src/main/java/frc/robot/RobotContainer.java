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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;



/* Subsystem Imports */
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

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
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final AlgaeSubsystem m_Algae = new AlgaeSubsystem();
  // TODO: climb system manipulator here.

  /* TODO: Auto stuff here.  Line 69-99 in 2024 code. */


  /* The controller that are used to control the robot.  Initialized here. */
  CommandXboxController m_DriverController = new CommandXboxController( OIConstants.kDriverControllerPort );
  CommandXboxController m_OperatorController = new CommandXboxController( OIConstants.k2ndDriverControllerPort );
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
        -MathUtil.applyDeadband(m_DriverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_DriverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_DriverController.getRightX(), OIConstants.kDriveDeadband),
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
    /************************* DriveTrain Commands ****************************/
    /* Command to set wheels in X formation when right stick gets pushed down. */
    m_DriverController.rightStick().whileTrue( new RunCommand( () -> m_robotDrive.setX(), m_robotDrive ));

    /* Command to reset the robot heading when "start" gets pushed. */
    m_DriverController.start().onTrue(new InstantCommand( m_robotDrive::zeroHeading ).ignoringDisable(true));

    /************************** Elevator Commands *****************************/
    /* Command to run the elevator with the left joystick of the op controller. */
    /* *****Note: Must hold left trigger to do so. */
    m_OperatorController.leftTrigger().whileTrue( m_Elevator.ElevatorManualCmd( m_OperatorController )  );

    /* Command to reset the elevator encoders. */
    m_OperatorController.povLeft().whileTrue( new InstantCommand( m_Elevator::resetElevatorPosition ) );

    /* Command to run elevator up with POV hat up. */
    //m_OperatorController.povUp().whileTrue( m_Elevator.ElevatorManualUp( .1 ) );

    /* Command to run the elevator down with POV hat down. */
    //m_OperatorController.povDown().whileTrue( m_Elevator.ElevatorManualDown( .1 ) );




    /*************************** Algae Commands *******************************/
    m_OperatorController.x().whileTrue( m_Algae.IntakeAlgaeForwardCommand() );
    m_OperatorController.x().whileFalse( m_Algae.IntakeAlgaeStopCommand() );

    m_OperatorController.y().whileTrue( m_Algae.IntakeAlgaeReverseCommand() );
    m_OperatorController.y().whileFalse( m_Algae.IntakeAlgaeStopCommand() );


    /*************************** Coral Commands *******************************/

    m_OperatorController.rightTrigger().whileTrue(m_coral.CoralPivotCmd( m_OperatorController ) );

    /*************************** Climb Commands *******************************/



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
