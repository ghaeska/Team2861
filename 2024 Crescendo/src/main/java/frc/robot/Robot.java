// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

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
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterArmState;
import frc.utils.Helpers;
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
public class Robot extends TimedRobot
{
  private Command m_autonomousCommand;

  /* Automatic turning functionality */
  private boolean m_TurnToAngle;
  private double m_yaw;
  private double currentRotationalRate;
  private final PIDController m_PIDController = new PIDController( Constants.DriveConstants.k_turnPID_P,
                                                                   Constants.DriveConstants.k_turnPID_I,
                                                                   Constants.DriveConstants.k_turnPID_D
                                                                 );
  
  public static final Pigeon2 m_gyro = new Pigeon2( DriveConstants.kGyroCanId );

  /* Controller */
  XboxController m_DriverController = new XboxController( OIConstants.kDriverControllerPort );

  /* Robot Subsytems */
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final Intake m_Intake = Intake.getInstance();
  private final Shooter m_Shooter = Shooter.getInstance();
  private final DriveTrain m_DriveTrain = DriveTrain.getInstance();

  private IntakeState stateIntake = IntakeState.NONE;

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
    m_allSubsystems.add( m_Shooter );
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
    m_PIDController.setTolerance( Constants.DriveConstants.k_tolerance_degrees );
    m_PIDController.enableContinuousInput( -180.0, 180.0 );    
  }
  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    m_TurnToAngle = false;
    
    /* Check controller for Drive Commands */
    if( m_DriverController.getRightStickButton() )
    {
      //m_DriveTrain.zeroHeading();
    }
    else if( m_DriverController.getLeftStickButton() )
    {
      m_DriveTrain.setX();
    }
    /* POV is the DPAD, UP=0, DOWN=180, LEFT=270, RIGHT=90 */
    else if(m_DriverController.getPOV() == 0 )
    {
      /* Idea, press a button, feed that into the drive, then manually rotate until it hits that spot */
      m_PIDController.setSetpoint( 0.0 ); //UP
      m_TurnToAngle = true;
    }
    else if(m_DriverController.getPOV() == 90 ) //Right
    {
      m_PIDController.setSetpoint( 90.0 );
      m_TurnToAngle = true;
    }
    else if(m_DriverController.getPOV() == 180 ) //Down
    {
      m_PIDController.setSetpoint( 179.9 );
      m_TurnToAngle = true;
    }
    else if(m_DriverController.getPOV() == 270 ) //Left 
    {
      m_PIDController.setSetpoint( -90.0 );
      m_TurnToAngle = true;
    }    

    if( m_TurnToAngle )
    {
      m_yaw = m_gyro.getAngle();
      double pid_output = m_PIDController.calculate( m_yaw );
      pid_output = Helpers.clamp( pid_output, -1.0, 1.0 );
      currentRotationalRate = pid_output;

      m_DriveTrain.drive( -MathUtil.applyDeadband( -m_DriverController.getLeftX(), OIConstants.kDriveDeadband ),
                          -MathUtil.applyDeadband( m_DriverController.getLeftY(), OIConstants.kDriveDeadband ),
                          currentRotationalRate,
                          true, 
                          true );
    }

    if( m_DriverController.getRightX() != 0 )
    {
      m_TurnToAngle = false;
      /* Control the rotation by joystick again.  */
      m_DriveTrain.drive( -MathUtil.applyDeadband( -m_DriverController.getLeftX(), OIConstants.kDriveDeadband ),
                          -MathUtil.applyDeadband( m_DriverController.getLeftY(), OIConstants.kDriveDeadband ),
                          -MathUtil.applyDeadband( m_DriverController.getRightX(), OIConstants.kDriveDeadband ),
                          true, 
                          true );
    }

    



    // m_DriveTrain.drive( -MathUtil.applyDeadband( -m_DriverController.getLeftX(), OIConstants.kDriveDeadband ),
    //                     -MathUtil.applyDeadband( m_DriverController.getLeftY(), OIConstants.kDriveDeadband ),
    //                     -MathUtil.applyDeadband( m_DriverController.getRightX(), OIConstants.kDriveDeadband ),
    //                     true, 
    //                     true );


    /* Check controller for Intake commands */
    if( m_DriverController.getAButtonPressed() )
    {
      /* 
      ** If we are going in reverse, we dont want to just change direction, might
      ** be hard on the motor/gearbox.  We will want to stop our Intake before we 
      ** change its direction.
      */
      if( stateIntake == IntakeState.EJECT )
      {
        /* We are spinning for ejection, stop before we intake */
        stateIntake = IntakeState.NONE;
      }
      else
      {
        /* We arent doing anything, go ahead and intake. */
        stateIntake = IntakeState.INTAKE_SLOW;
      }
    }
    else if( m_DriverController.getBButtonPressed() )
    {
      /* 
      ** If we are going in reverse, we dont want to just change direction, might
      ** be hard on the motor/gearbox.  We will want to stop our Intake before we 
      ** change its direction.
      */
      if( stateIntake == IntakeState.INTAKE_SLOW || stateIntake == IntakeState.INTAKE_FAST )
      {
        /* We are spinning for intake, stop before we eject */
        stateIntake = IntakeState.NONE;
      }
      else
      {
        /* We arent doing anything, go ahead and eject. */
        stateIntake = IntakeState.EJECT;
      }
    }
    else if( m_DriverController.getYButtonPressed() )
    {
      /* stop all intake functionality. */
      stateIntake = IntakeState.NONE;
    }
    else if( m_DriverController.getXButtonPressed() )
    {
      /* 
      ** If we are going in reverse, we dont want to just change direction, might
      ** be hard on the motor/gearbox.  We will want to stop our Intake before we 
      ** change its direction.
      */
      if( stateIntake == IntakeState.EJECT )
      {
        /* We are spinning for ejection, stop before we intake */
        stateIntake = IntakeState.NONE;
      }
      else
      {
        /* We arent doing anything, go ahead and intake. */
        stateIntake = IntakeState.INTAKE_FAST;
      }
    }
    else if( m_DriverController.getLeftBumperPressed() )
    {
      m_Shooter.setShooterArmTarget( ShooterArmState.SPEAKER );
    }
    else if( m_DriverController.getRightBumperPressed() )
    {
      m_Shooter.setShooterArmTarget( ShooterArmState.STOWED );
    }
    else 
    {
      
    }

    /* Update Intake state after input */
    m_Intake.setState( stateIntake );

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
