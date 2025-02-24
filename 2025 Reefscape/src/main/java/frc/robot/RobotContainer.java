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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;



/* Subsystem Imports */
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetpoint;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

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
public class RobotContainer 
{
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();

  private final AlgaeSubsystem m_Algae = new AlgaeSubsystem();
  private final LEDsSubsystem m_LED = new LEDsSubsystem();
  //private final ClimbSubsystem m_climb = new ClimbSubsystem();
  // TODO: climb system manipulator here.

  private SendableChooser<Command> autoChooser;


  /* The controller that are used to control the robot.  Initialized here. */
  CommandXboxController m_DriverController = new CommandXboxController( OIConstants.kDriverControllerPort );
  CommandXboxController m_OperatorController = new CommandXboxController( OIConstants.k2ndDriverControllerPort );

  private void registerNamedCommands()
  {
    /* Add named commands here so that they can be used in autos. */
    NamedCommands.registerCommand("Elevator stow", m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_stow ) );
    NamedCommands.registerCommand("Elevator L1", m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l1 ) );
    NamedCommands.registerCommand("Elevator L2", m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l2 ) );
    NamedCommands.registerCommand("Elevator L3", m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l3 ) );
    //NamedCommands.registerCommand("Elevator L4", m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l4 ) );    
    NamedCommands.registerCommand("Elevator Feeder", m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_feederStation ) );

    



  }



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    /* Call the command to get the registered commands. */
    registerNamedCommands();

    /* Create a Auto Selector */
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selector", autoChooser );

    /* Configure the button bindings */
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand( () -> m_robotDrive.drive(
        getDriveForward(),
        getDriveStrafe(),
        getDriveRotation(),
        true,
        false),
        m_robotDrive ) );

  }
  private double getDriveStrafe()
  {
    double controllerStrafe = -MathUtil.applyDeadband(m_DriverController.getLeftX(), OIConstants.kDriveDeadband);
    if( m_DriverController.leftTrigger().getAsBoolean() == true )
    {
      //check to see if the area is large enough to assume we are lined up.
      double m_strafe = m_vision.getLimelightTA();
      if( m_strafe >= 9.5 )
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

    /************************** Elevator Commands *****************************/

    /* Command to reset the elevator encoders. */
    m_OperatorController.back().whileTrue( new InstantCommand( m_Elevator::resetElevatorPosition ).ignoringDisable(true) );
    /* POV up, run to L4 */
    //m_OperatorController.povUp().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_l4 ) );
    m_OperatorController.povUp().onTrue( Commands.sequence( m_Elevator.setElevatorSetpointCmd(ElevatorSetpoint.k_l4_up ),
                                                            Commands.waitSeconds( 1.0 ),
                                                            m_Elevator.setElevatorSetpointCmd(ElevatorSetpoint.k_l4_score ) ) );
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

    m_OperatorController.a().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_LowA ) );
    m_OperatorController.b().onTrue( m_Elevator.setElevatorSetpointCmd( ElevatorSetpoint.k_HighA ) );
    
    /*************************** Algae Commands *******************************/
    m_OperatorController.x().whileTrue( m_Algae.IntakeAlgaeForwardCommand() );
    m_OperatorController.x().whileFalse( m_Algae.IntakeAlgaeStopCommand() );

    m_OperatorController.y().whileTrue( m_Algae.IntakeAlgaeReverseCommand() );
    m_OperatorController.y().whileFalse( m_Algae.IntakeAlgaeStopCommand() );


    /*************************** Coral Commands *******************************/
    /* Intake with left bumper for Left Coral */
    m_DriverController.leftBumper().whileTrue
    ( 
      Commands.sequence
      ( 
        m_coral.CoralRunMotorCmd( .2, m_LED ),
        m_LED.LED_LgreenRredCmd()
      )    
    );
    m_DriverController.leftBumper().whileFalse( m_coral.CoralRunMotorCmd( 0.0, m_LED ) );

    /* Intake with right bumper for Right Coral */
    m_DriverController.rightBumper().whileTrue
    ( 
      Commands.sequence
      (
        m_coral.CoralRunMotorCmd( .2, m_LED ),
        m_LED.LED_LredRgreenCmd()
      )
    );
    m_DriverController.rightBumper().whileFalse( m_coral.CoralRunMotorCmd( 0.0, m_LED ) );

    /* Outake the coral. */
    m_DriverController.rightTrigger().whileTrue( m_coral.CoralRunMotorCmd( -.2, m_LED) );
    m_DriverController.rightTrigger().whileFalse( m_coral.CoralRunMotorCmd( 0.0, m_LED ) );




    /*************************** Climb Commands *******************************/



  } 

  // private void configureNewCommands()
  // {

  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  { 
    return autoChooser.getSelected();
  }

} 
