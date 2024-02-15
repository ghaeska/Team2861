package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utils.Helpers;

public class Shooter extends Subsystem
{
  /*-------------------- Private Instance Variables --------------------------*/
  private static Shooter m_Instance;
  private PeriodicIO m_PeriodicIO;

  private CANSparkFlex m_TopShooterMotor;
  private CANSparkFlex m_BotShooterMotor;

  private SparkPIDController m_TopShooterPID;
  private SparkPIDController m_BottomShooterPID;

  private RelativeEncoder m_TopShooterEncoder;
  private RelativeEncoder m_BottomShooterEncoder;

  private BangBangController BBController;
  
  public static Shooter getInstance() 
  {
    if (m_Instance == null)
    {
      m_Instance = new Shooter();
    }
    return m_Instance;
  }

  private Shooter()
  {
    m_PeriodicIO = new PeriodicIO();

    /* Setup the Motor Controllers */
    m_TopShooterMotor = new CANSparkFlex( Constants.Shooter.k_ShooterTopMotorCanId, MotorType.kBrushless );
    m_BotShooterMotor = new CANSparkFlex( Constants.Shooter.k_ShooterBotMotorCanId, MotorType.kBrushless );

    /* Restore them to defaults */
    m_TopShooterMotor.restoreFactoryDefaults();
    m_BotShooterMotor.restoreFactoryDefaults();

    /* Set the motors Idle Mode */
    m_TopShooterMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );
    m_BotShooterMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );

    /* Set the motor Inversions */
    m_TopShooterMotor.setInverted( true );
    m_BotShooterMotor.setInverted( false );

    /* Set the current limits on the Motors */
    m_BotShooterMotor.setSmartCurrentLimit( 40 );
    m_TopShooterMotor.setSmartCurrentLimit( 40 );

    /* Setup the Bang Bang Controller */
    BBController = new BangBangController();

    /* Setup the Motor Encoders */
    m_TopShooterEncoder = m_TopShooterMotor.getEncoder();
    m_BottomShooterEncoder = m_BotShooterMotor.getEncoder();

    /* Create a Leader/Follower Motor for Shooter System */
    m_BotShooterMotor.follow(m_TopShooterMotor,true);

    /* Burn the new configurations into the motor controllers flash. */
    m_BotShooterMotor.burnFlash();
    m_TopShooterMotor.burnFlash();
  }

  private static class PeriodicIO 
  {
    /* Value to set for shooter rpm */
    double shooter_rpm = 0.0;
    /* Enumeration for switching to set rpms. */
    ShooterState shootState = ShooterState.NONE;

  }

  /*-------------------- Generic Subsystem Functions -------------------------*/

  @Override
  public void periodic() 
  {
    /* Check to see if we have a note and need to verify position. */
    checkAutoTasks();

    /* Shooter Control */
    m_PeriodicIO.shooter_rpm = ShooterStateToSpeed( m_PeriodicIO.shootState );
    
  }

  @Override
  public void writePeriodicOutputs() 
  {
    /* 
    ** Because our bottom motor is set to follow the top motor, we should only
    ** need to set the speed of our top motor. 
    */
    m_TopShooterMotor.set( BBController.calculate (m_TopShooterEncoder.getVelocity(), m_PeriodicIO.shooter_rpm ));
  }

  @Override
  public void stop() 
  {
    stopShooter();
  }

  @Override
  public void outputTelemetry() 
  {
    SmartDashboard.putNumber("Shooter speed (RPM):", m_PeriodicIO.shooter_rpm);
    SmartDashboard.getNumber("Set Shooter Speed", m_PeriodicIO.shooter_rpm ); // See if this allows for easy speed changes.
    SmartDashboard.putNumber("Shooter left speed:", m_TopShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter right speed:", m_BottomShooterEncoder.getVelocity());

    SmartDashboard.putNumber("Shooter Set Speed:", ShooterStateToSpeed(m_PeriodicIO.shootState));
  }

  @Override
  public void reset() 
  {

  }

  /*---------------------- Custom Public Functions ---------------------------*/
  public enum ShooterState
  {
    NONE,
    SPEAKER,
    AMP,
    HOLD,
    PASS,
    STAGE
  }

  public double ShooterStateToSpeed( ShooterState state ) 
  {
    switch( state ) 
    {
      case NONE:
        return 0.0;
      case SPEAKER:
        return Constants.Shooter.k_ShooterSpeed_Speaker;
      case AMP:
        return Constants.Shooter.k_ShooterSpeed_Amp;
      case HOLD:
        return 0.0;
      case PASS:
        return Constants.Shooter.k_ShooterSpeed_Pass;
      case STAGE:
        return Constants.Shooter.k_ShooterSpeed_Stage;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  public void setSpeed( double rpm ) 
  {
    m_PeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() 
  {
    m_PeriodicIO.shooter_rpm = 0.0;
    m_PeriodicIO.shootState = ShooterState.NONE;
  }

  public void setShooterArmTarget( ShooterState target ) 
  {
    m_PeriodicIO.shootState = target;
  }  

  /*---------------------- Custom Private Functions --------------------------*/
  private void checkAutoTasks() 
  {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    // if( m_PeriodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && isPivotAtTarget() ) 
    // {
    //   m_PeriodicIO.pivot_target = PivotTarget.STOW;    
    // }
  }
}