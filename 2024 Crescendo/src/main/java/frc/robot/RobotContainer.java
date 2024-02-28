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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.Index;
//import frc.utils.CommandXboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.Map;

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
  private final DriveSubsystem    m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem   m_intake     = new IntakeSubsystem();
  private final ShooterSubsystem  m_shooter    = new ShooterSubsystem();
  private final ArmSubsystem      m_arm        = new ArmSubsystem();
  private final IndexSubsystem    m_index      = new IndexSubsystem();

  private SendableChooser<Command> autoChooser;// = new SendableChooser<>();
  private SendableChooser<String> OverrideChooser = new SendableChooser<>();

  //private String m_lastControl = "Commands Are Auto";

  private void configureAutoCommands()
  {
    NamedCommands.registerCommands( Map.of
    (
      "StowArm", Commands.sequence(
        m_arm.StowArmCommand(),
        Commands.waitSeconds(.125)
      ),
      "StartShooter", Commands.sequence(
        m_shooter.runShooterDefaultSpeedCommand()
      ),
      "RunIntakeIndex", Commands.parallel(
        m_intake.runIntakeSlowCommand(),
        m_index.runIndexFwdCommand()
      ),
      "StopIntakeIndex", Commands.parallel(
        m_index.stopIndexCommand(),
        m_intake.stopIntakeCommand()
      ),
      "SpeakerShootSequence", Commands.sequence(
        m_shooter.runShooterSpeakerCommand(),
        Commands.waitSeconds(.5),
        m_index.runIndexShootCommand()       
      )
    ) );
  }
  

 
  /* Initialize a controller that is plugged into the defined drive controller port. */
  CommandXboxController m_OperatorController = new CommandXboxController( OIConstants.k2ndDriverControllerPort );
  CommandXboxController m_xboxController = new CommandXboxController( OIConstants.kDriverControllerPort );
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    /* Check to see if we want to override the auto commands */
    OverrideChooser.setDefaultOption( "Commands Are Auto", "Commands Are Auto" );
    OverrideChooser.addOption( "Manual Control", "Manual Control" );
    SmartDashboard.putData( "Command Override Chooser", OverrideChooser );

    /* Configure the button bindings, as defined. */ 
    configureButtonBindings();

    
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband( m_xboxController.getLeftY(), OIConstants.kDriveDeadband ),
              -MathUtil.applyDeadband( m_xboxController.getLeftX(), OIConstants.kDriveDeadband ),
              -MathUtil.applyDeadband( m_xboxController.getRightX(), OIConstants.kDriveDeadband ),
              true, 
              true),
              m_robotDrive ) );

    configureAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    // m_arm.setDefaultCommand( m_arm.defaultCommand( () -> MathUtil.applyDeadband
    //                                                      (
    //                                                       m_OperatorController.getLeftY(),
    //                                                       0.2 ) ) );                                  
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
    // /* Configure a trigger to change the control style when a selection is made. */
    // Trigger controlPick = new Trigger( () -> m_lastControl != OverrideChooser.getSelected() );
    // controlPick.onTrue( runOnce( () -> configureNewCommands() ) );

    /* DriveTrain Commands */
    m_xboxController.rightStick().whileTrue( new RunCommand( () -> m_robotDrive.setX(), m_robotDrive ));
    m_xboxController.start().onTrue(new InstantCommand( m_robotDrive::ResetYaw ).ignoringDisable(true));

    /* Arm Commands */
    m_xboxController.x().onTrue( m_arm.StowArmCommand() );
    m_xboxController.y().onTrue( m_arm.AmpArmCommand() );
    // m_xboxController.a().onTrue( m_arm.runArmCommand() );
    // m_xboxController.b().onTrue( m_arm.stopArmCommand() );
    // m_xboxController.y().onTrue( m_arm.runArmRevCommand() );   

    /* Shooter Commands */
    m_OperatorController.rightTrigger().onTrue( m_shooter.runShooterSpeakerCommand() );
    m_OperatorController.leftTrigger().onTrue( m_shooter.runShooterStopCommand() );

    /* Intake Commands */
    m_OperatorController.x().onTrue( m_intake.runIntakeFastCommand() );
    m_OperatorController.y().onTrue( m_intake.ejectIntakeCommand() );
    m_OperatorController.a().onTrue( m_intake.runIntakeSlowCommand() );
    m_OperatorController.b().onTrue( m_intake.stopIntakeCommand() );

    /* This command should run until the Intake sensor Triggers. */
    // m_OperatorController.x().onTrue( new RunCommand( () -> m_intake.runIntakeSlowCommand().until( m_intake.IntakeNoteSensor ), m_intake ));

    
    /* Indexer Commands */
    m_OperatorController.rightBumper().whileTrue( new RunCommand( ()-> m_index.runIndex( Index.k_IndexForwardSpeed ), m_index) );
    m_OperatorController.rightBumper().whileFalse( m_index.stopIndexCommand() );  
    m_OperatorController.leftBumper().whileTrue( m_index.runIndexRevCommand() );
    m_OperatorController.leftBumper().whileFalse( m_index.stopIndexCommand() );

    /* Setup Automation Commands */
    /* Turn on the indexer when the intake senses a note. */
    // m_intake.IntakeNoteSensor.onFalse( m_index.runIndexFwdCommand() ); 
    
    // /* Turn on the Intake once the note leaves the shooter, with a slight delay */
    // m_index.IndexNoteSensor.onFalse( Commands.sequence( 
    //                                                     Commands.waitSeconds(.5),
    //                                                     m_intake.runIntakeSlowCommand() 
    //                                                   )
    //                                );
    if( !m_intake.getIntakeSensor() )
    {
      m_intake.runIntakeSlowCommand();
    }

    m_intake.runIntakeSlowCommand();

    //m_index.IndexNoteSensor.whileFalse(m_intake.runIntakeSlowCommand() );

 
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