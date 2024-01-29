package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;
//import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends Subsystem
{
  /*-------------------- Private Instance Variables --------------------------*/
  private static Shooter m_Instance;
  private PeriodicIO m_PeriodicIO;

  private CANSparkMax m_LeftShooterMotor;
  private CANSparkMax m_RightShooterMotor;

  private SparkPIDController m_LeftShooterPID;
  private SparkPIDController m_RightShooterPID;

  private RelativeEncoder m_LeftShooterEncoder;
  private RelativeEncoder m_RightShooterEncoder;

  private SlewRateLimiter m_ShooterSlewLimiter = new SlewRateLimiter( 1000 );
  
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
    m_LeftShooterMotor = new CANSparkMax( Constants.Shooter.k_ShooterLeftMotorCanId, MotorType.kBrushless );
    m_RightShooterMotor = new CANSparkMax( Constants.Shooter.k_ShooterRightMotorCanId, MotorType.kBrushless );

    /* Restore them to defaults */
    m_LeftShooterMotor.restoreFactoryDefaults();
    m_RightShooterMotor.restoreFactoryDefaults();

    /* Set the motors Idle Mode */
    m_LeftShooterMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );
    m_RightShooterMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );

    /* Set the motor Inversions */
    m_LeftShooterMotor.setInverted( false );  // GTH:TODO need to update
    m_RightShooterMotor.setInverted( false ); // GTH:TODO need to update

    /* Setup the PID Controllers */
    m_LeftShooterPID = m_LeftShooterMotor.getPIDController();
    m_LeftShooterPID.setP( Constants.Shooter.k_ShooterMotorP );
    m_LeftShooterPID.setI( Constants.Shooter.k_ShooterMotorI );
    m_LeftShooterPID.setD( Constants.Shooter.k_ShooterMotorD );
    m_LeftShooterPID.setFF( Constants.Shooter.k_ShooterMotorFF );
    m_LeftShooterPID.setOutputRange( Constants.Shooter.k_ShooterMinOutput, Constants.Shooter.k_ShooterMaxOutput );

    m_RightShooterPID = m_RightShooterMotor.getPIDController();
    m_RightShooterPID.setP( Constants.Shooter.k_ShooterMotorP );
    m_RightShooterPID.setI( Constants.Shooter.k_ShooterMotorI );
    m_RightShooterPID.setD( Constants.Shooter.k_ShooterMotorD );
    m_RightShooterPID.setFF( Constants.Shooter.k_ShooterMotorFF );
    m_RightShooterPID.setOutputRange( Constants.Shooter.k_ShooterMinOutput, Constants.Shooter.k_ShooterMaxOutput );

    /* Setup the Motor Encoders */
    m_LeftShooterEncoder = m_LeftShooterMotor.getEncoder();
    m_RightShooterEncoder = m_RightShooterMotor.getEncoder();
  }

  private static class PeriodicIO 
  {
    double shooter_rpm = 0.0;
  }

  /*-------------------- Generic Subsystem Functions -------------------------*/

  @Override
  public void periodic() 
  {

  }

  @Override
  public void writePeriodicOutputs() 
  {
    double limitedSpeed = m_ShooterSlewLimiter.calculate(m_PeriodicIO.shooter_rpm);
    m_LeftShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
    m_RightShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
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
    SmartDashboard.putNumber("Shooter left speed:", m_LeftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter right speed:", m_RightShooterEncoder.getVelocity());
  }

  @Override
  public void reset() 
  {

  }

  /*---------------------- Custom Public Functions ---------------------------*/

  public void setSpeed( double rpm ) 
  {
    m_PeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() 
  {
    m_PeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------- Custom Private Functions --------------------------*/

}
