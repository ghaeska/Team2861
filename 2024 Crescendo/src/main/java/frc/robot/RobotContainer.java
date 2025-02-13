// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import edu.wpi.first.wpilibj.XboxController.Button;
//import frc.robot.Constants.Index;
//import frc.utils.CommandXboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import frc.robot.SwerveConstants.AutoConstants;
//import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeIndexSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.IntakeNote;
import frc.robot.commands.PrepareShootNote;
import frc.robot.commands.FeedNote;
import frc.robot.commands.DriveToIndex;
import frc.robot.commands.IntakeToDrive;

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
  private final DriveSubsystem    m_robotDrive = new DriveSubsystem();
  //private final IntakeSubsystem   m_intake     = new IntakeSubsystem();
  private final ShooterSubsystem  m_shooter    = new ShooterSubsystem();
  private final ArmSubsystem      m_arm        = new ArmSubsystem();
  //private final IndexSubsystem    m_index      = new IndexSubsystem();
  private final IntakeIndexSubsystem m_indexIntake = new IntakeIndexSubsystem();

  //public static SendableChooser<Integer> autoChooser = new SendableChooser<>();  
  private SendableChooser<Command> autoChooser;
 
  /* Initialize a controller that is plugged into the defined drive controller port. */
  CommandXboxController m_OperatorController = new CommandXboxController( OIConstants.k2ndDriverControllerPort );
  CommandXboxController m_driverController = new CommandXboxController( OIConstants.kDriverControllerPort );
  
  private void registerNamedCommands()
  {
    NamedCommands.registerCommand( "IntakeNote", ( new IntakeNote( m_indexIntake ) ) );
    NamedCommands.registerCommand( "ShootNote", ( new PrepareShootNote(m_indexIntake, m_shooter)));
    NamedCommands.registerCommand("FeedNote", (new FeedNote(m_indexIntake)));
    NamedCommands.registerCommand("IntakeToDrive",( new IntakeToDrive(m_indexIntake)) );
    NamedCommands.registerCommand("DriveToIndex", ( new DriveToIndex(m_indexIntake)) );
    NamedCommands.registerCommand( "StowArm", 
                                  Commands.sequence( 
                                  m_arm.StowArmCommand(),
                                  Commands.waitSeconds(.125) ) );
    NamedCommands.registerCommand( "StartShooter", 
                                  //Commands.sequence(
                                  m_shooter.runShooterSpeakerCommand() );
                                  //Commands.waitSeconds(2) ) );
    NamedCommands.registerCommand( "FeedShooter2Index",
                                  Commands.sequence( 
                                  m_indexIntake.runIndexFwdCommand(),
                                  Commands.waitSeconds(1),
                                  m_indexIntake.stopIndexCommand() ) );
    NamedCommands.registerCommand( "StopIntakeIndex",
                                  Commands.sequence(
                                  m_indexIntake.stopIndexCommand(),
                                  m_indexIntake.stopIntakeCommand() ) );
    NamedCommands.registerCommand( "SpeakerShootSequence", 
                                  Commands.sequence(
                                  m_shooter.runShooterSpeakerCommand(),
                                  Commands.waitSeconds(.5),
                                  m_indexIntake.feedShooterFromIndexCommand() ) );
   //NamedCommands.registerCommand(  ) );
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    /* Configure the button bindings, as defined. */ 
    configureButtonBindings();    
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband( m_driverController.getLeftY(), OIConstants.kDriveDeadband ),
              -MathUtil.applyDeadband( m_driverController.getLeftX(), OIConstants.kDriveDeadband ),
              -MathUtil.applyDeadband( m_driverController.getRightX(), OIConstants.kDriveDeadband ),
              true, 
              true),
              m_robotDrive ) );
    
    m_arm.setDefaultCommand( m_arm.defaultCommand( () -> MathUtil.applyDeadband
                                                         (
                                                          m_OperatorController.getLeftY(),
                                                          0.2 ) ) );                                  
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
    /* DriveTrain Commands */
    m_driverController.rightStick().whileTrue( new RunCommand( () -> m_robotDrive.setX(), m_robotDrive ));
    m_driverController.start().onTrue(new InstantCommand( m_robotDrive::ResetYaw ).ignoringDisable(true));

    /* Intake Commands */
    // m_OperatorController.x().onTrue( m_intake.runIntakeFastCommand() );
    m_OperatorController.y().onTrue( m_indexIntake.ejectIntakeCommand() );
    m_OperatorController.a().onTrue( m_indexIntake.runIntakeSlowCommand() );
    // m_OperatorController.b().onTrue( m_intake.stopIntakeCommand() );

    // Need to see which ones of these commands works for the automation.
    //m_OperatorController.a().onTrue( m_indexIntake.IntakeToIndexCommand() );

    m_OperatorController.b().onTrue( new IntakeNote( m_indexIntake ) );

    //m_OperatorController.y().onTrue( m_indexIntake.feedShooterFromIndexCommand() );

    /* Arm Commands */    
    m_OperatorController.povDown().onTrue( m_arm.StowArmCommand() );
    m_OperatorController.povUp().onTrue( m_arm.AmpArmCommand() );
    m_OperatorController.povLeft().onTrue( m_arm.HangArmCommand() );


    /* Shooter Commands */
    m_OperatorController.rightTrigger().onTrue( m_shooter.runShooterSpeakerCommand() );
    m_OperatorController.leftTrigger().onTrue( m_shooter.runShooterStopCommand() );

    /* Indexer Commands */
    m_OperatorController.rightBumper().whileTrue( new RunCommand( ()-> m_indexIntake.runIndex( Constants.Index.k_IndexForwardSpeed ), m_indexIntake) );
    m_OperatorController.rightBumper().whileFalse( m_indexIntake.stopIndexCommand() );  
    m_OperatorController.leftBumper().whileTrue( m_indexIntake.runIndexRevCommand() );
    m_OperatorController.leftBumper().whileFalse( m_indexIntake.stopIndexCommand() );
    
  }

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