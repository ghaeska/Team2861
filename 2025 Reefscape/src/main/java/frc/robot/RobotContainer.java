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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;



/* Subsystem Imports */
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetpoint;
//import frc.robot.subsystems.AlgaeSubsystem;
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
  //private final AlgaeSubsystem m_Algae = new AlgaeSubsystem();
  //private final LEDsSubsystem m_LED = new LEDsSubsystem();
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

    //m_Elevator.setDefaultCommand(m_Elevator.defaultCommand() );
  }

  /* Configure all the buttons for the controller. */
  private void configureButtonBindings() 
  {
    /************************* DriveTrain Commands ****************************/
    /* Command to set wheels in X formation when right stick gets pushed down. */
    m_DriverController.rightStick().whileTrue( new RunCommand( () -> m_robotDrive.setX(), m_robotDrive ));

    /* Command to reset the robot heading when "start" gets pushed. */
    m_DriverController.start().onTrue(new InstantCommand( m_robotDrive::zeroHeading ).ignoringDisable(true));

    /************************** Elevator Commands *****************************/

    /* Command to reset the elevator encoders. */
    m_OperatorController.back().whileTrue( new InstantCommand( m_Elevator::resetElevatorPosition ).ignoringDisable(true) );

    /* POV up, run to L4 */
    m_OperatorController.povUp().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l4 ) );
    /* POV right, run to L3 */
    m_OperatorController.povRight().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l3 ) );
    /* POV down, run to L2 */
    m_OperatorController.povDown().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l2 ) );
    /* POV left, run to l1 */
    m_OperatorController.povLeft().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l1 ) );
    /* left bumper, run to the stow positon. */
    m_OperatorController.leftBumper().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_stow ) );
    /* right bumper, run to the feeder position */
    m_OperatorController.rightBumper().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_feederStation ) );

    

    
    /*************************** Algae Commands *******************************/
    // m_OperatorController.x().whileTrue( m_Algae.IntakeAlgaeForwardCommand() );
    // m_OperatorController.x().whileFalse( m_Algae.IntakeAlgaeStopCommand() );

    // m_OperatorController.y().whileTrue( m_Algae.IntakeAlgaeReverseCommand() );
    // m_OperatorController.y().whileFalse( m_Algae.IntakeAlgaeStopCommand() );


    /*************************** Coral Commands *******************************/

    m_OperatorController.a().whileTrue( m_coral.CoralRunMotorCmd( .4 ) );
    m_OperatorController.a().whileFalse( m_coral.CoralRunMotorCmd( 0 ) );

    m_OperatorController.b().whileTrue( m_coral.CoralRunMotorCmd( -.4) );
    m_OperatorController.b().whileFalse( m_coral.CoralRunMotorCmd( 0 ) );


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
