// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;

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
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;





/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private Command m_autonomousCommand;
  //private RobotContainer m_robotContainer;

  /* Controller */
  XboxController m_xboxController = new XboxController( OIConstants.kDriverControllerPort );

  /* Robot Subsytems */
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final Intake m_Intake = Intake.getInstance();
  private final DriveTrain m_DriveTrain = DriveTrain.getInstance();

  /* Auto Stuff */
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();
  private AutoChooser m_autoChooser = new AutoChooser();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_allSubsystems.add( m_DriveTrain );
    m_allSubsystems.add( m_Intake );
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    //CommandScheduler.getInstance().run();
    m_allSubsystems.forEach(subsystem -> subsystem.periodic() );
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs() );
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry() );
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog() );
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    m_allSubsystems.forEach(subsystem -> subsystem.stop() );
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() 
  {
    m_autoRunner.setAutoMode(m_autoChooser.getSelectedAuto());
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if(m_currentTask != null) 
    {
      m_currentTask.start();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    // If there is a current task, run it
    if (m_currentTask != null) 
    {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) 
      {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) 
        {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() 
  {    
    /* Nothing */
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    m_DriveTrain.drive( -MathUtil.applyDeadband( -m_xboxController.getLeftX(), OIConstants.kDriveDeadband ),
                        -MathUtil.applyDeadband( m_xboxController.getLeftY(), OIConstants.kDriveDeadband ),
                        -MathUtil.applyDeadband( m_xboxController.getRightX(), OIConstants.kDriveDeadband ),
                        true, 
                        true );
    /* Check controller for Drive Commands */
    if( m_xboxController.getRightStickButton() )
    {
      m_DriveTrain.zeroHeading();
    }
    else if( m_xboxController.getLeftStickButton() )
    {
      m_DriveTrain.setX();
    }

    /* Check controller for Intake commands */
    if( m_xboxController.getAButton() )
    {
      m_Intake.goToGround();
    }
    else if( m_xboxController.getBButton() )
    {
      if( m_Intake.getIntakeHasNote() )
      {
        m_Intake.pulse();
      }
      else
      {
        m_Intake.intake();
      }
    }
    else if( m_xboxController.getXButton() )
    {
      m_Intake.eject();
    }
    else if( m_xboxController.getLeftBumper() )
    {
      m_Intake.goToSource();
    }
    else if( m_xboxController.getRightBumper() )
    {
      m_Intake.goToStow();
    }
    else 
    {
      m_Intake.stopIntake();
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
