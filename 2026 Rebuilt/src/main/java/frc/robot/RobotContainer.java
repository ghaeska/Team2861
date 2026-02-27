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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.function.Supplier;

/* Subsystem Imports */
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShotCalcSubsystem;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSetpoint;
import frc.robot.subsystems.Vision.VisionSubsystem;

/* Pathplanner Imports */
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


/* Smartdashboard Imports */
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/*
** This class is where the bulk of the robot should be declared. Since Command-based is a
** "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
** periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
** subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer 
{
  /*************************** Subsytem initilization *****************/
  /* Initilize the drive subsystem. */
  DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  /* Initilize the Vision subsystem. */
  VisionSubsystem m_vision = new VisionSubsystem();
  //private final VisionSubsystem m_vision = new VisionSubsystem();
  /* Initilize the shooter subsystem. */
  ShooterSubsystem m_shooter = new ShooterSubsystem();
  /* Initilize the shooter hood subsystem. */
  ShooterHoodSubsystem m_shooterHood = new ShooterHoodSubsystem();
  /* Initilize the hopper subsystem. */
  HopperSubsystem m_hopper = new HopperSubsystem();
  /* Initilize the intake subsystem. */
  IntakeSubsystem m_intake = new IntakeSubsystem();
  /* Initilize the shot calculator subsystem. */
  ShotCalcSubsystem m_shotCalc = new ShotCalcSubsystem( m_robotDrive );
  /* Initilize the SuperStructure subsystem. */
  SuperSubsystem m_superSubsystem = new SuperSubsystem( m_robotDrive,
                                                        m_shooter,
                                                        m_shooterHood,
                                                        m_hopper,
                                                        m_intake,
                                                        m_shotCalc );

  /* Create the Autochooser on the new smartdashboard. */
  private SendableChooser<Command> autoChooser;

  /* Make our shooting target the center of the hub. */
  private final Pose3d target = FieldConstants.Hub.CENTER;
  private final Supplier<Double> distance = () -> m_robotDrive.getPose().getTranslation()
            .getDistance(target.getTranslation().toTranslation2d());

  /* The controller that are used to control the robot.  Initialized here. */
  CommandXboxController m_DriverController = new CommandXboxController( OIConstants.kDriverControllerPort );
  //CommandXboxController m_OperatorController = new CommandXboxController( OIConstants.k2ndDriverControllerPort );

  /* Named Commands used in Auto */
  private void registerNamedCommands()
  {
    //This is blank for now, until we make commands to be used with pathplanner.
    NamedCommands.registerCommand( "Drive To Target", m_robotDrive.DriveToTargetCommand( m_vision ).withDeadline(Commands.waitSeconds(1.4) ) ); 
    NamedCommands.registerCommand("DriveStop", m_robotDrive.DriveStop().withDeadline(Commands.waitSeconds( 0.01)));
    
  
  }

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    /* Call the command to get the registered commands. */
    registerNamedCommands();

    /* Create an Auto Selector */
    // Once we have more code added and start doing more Auto Routines, we will need to add a AutoChooser.
    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Selector", autoChooser );

    /* Configure the button bindings */ 
    configureButtonBindings();


    /* Add Default Commands here. */
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand( () -> m_robotDrive.drive(
        m_robotDrive.getDriveForward( m_DriverController, m_vision ),
        m_robotDrive.getDriveStrafe( m_DriverController, m_vision ),
        m_robotDrive.getDriveRotation( m_DriverController, m_vision ),
        m_robotDrive.DriveFieldRelative( m_DriverController ),
        false),
        m_robotDrive ) );
  }


  /* Configure all the buttons for the controller. */
  private void configureButtonBindings() 
  {
    /************************* DriveTrain Commands ****************************/
    /* Command to set wheels in X formation when right stick gets pushed down. */
    m_DriverController.rightStick().whileTrue( new RunCommand( () -> m_robotDrive.setX(), m_robotDrive ));

    /* Command to reset the robot heading when "start" gets pushed. */
    m_DriverController.start().onTrue(new InstantCommand( m_robotDrive::zeroHeading ).ignoringDisable(true) );
    //m_OperatorController.start().onTrue( new InstantCommand( m_robotDrive::zeroHeading ).ignoringDisable(true) );
    
    //Supplier<Double> velocity = 5000;
    m_DriverController.a().whileTrue( m_shooter.setShooterSetpointCmd( ShooterSetpoint.k_Shoot ) );
    m_DriverController.b().whileTrue( m_shooter.setShooterSetpointCmd( ShooterSetpoint.k_Stop ));


    m_DriverController.x().whileTrue(( m_intake.runIntakeCommand()));
    //m_DriverController.y().whileTrue(( m_intake.))
    // m_DriverController.b().onTrue( m_shooter.setVoltage(6) );
    // m_DriverController.b().whileFalse( m_shooter.setVoltage(0));

    //m_DriverController.x().whileTrue( m_shooter.ShooterForwardCommand() );
    //m_DriverController.x().whileFalse( m_shooter.ShooterStopCommand() );
    m_DriverController.leftBumper().whileTrue( m_hopper.revHopperCommand() );

    m_DriverController.leftTrigger().onTrue( m_shooter.decreaseFlywheelSpeedCmd() );
    m_DriverController.rightTrigger().onTrue( m_shooter.increaseFlywheelSpeedCmd() );
  }

  /* Send Commands to the Auto Chooser.  */
  public Command getAutonomousCommand() 
  {
    return autoChooser.getSelected();
  }
}
