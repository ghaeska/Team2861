package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;
//import frc.robot.subsystems.Intake.PivotTarget;
import frc.robot.subsystems.Intake.PivotTarget;
import frc.utils.Helpers;

public class Shooter extends Subsystem
{
  /*-------------------- Private Instance Variables --------------------------*/
  private static Shooter m_Instance;
  private PeriodicIO m_PeriodicIO;

  private CANSparkFlex m_TopShooterMotor;
  private CANSparkFlex m_BotShooterMotor;

  private CANSparkMax m_LeftShooterArmMotor;
  private CANSparkMax m_RightShooterArmMotor;


  private SparkPIDController m_LeftShooterPID;
  private SparkPIDController m_RightShooterPID;
  //private SparkPIDController m_ShooterArmPID;


  private RelativeEncoder m_LeftShooterEncoder;
  private RelativeEncoder m_RightShooterEncoder;

  private SlewRateLimiter m_ShooterSlewLimiter = new SlewRateLimiter( 1000 );

  private final PIDController m_ShooterArmPID = new PIDController( Constants.Shooter.k_ShooterArmMotorP,
                                                                  Constants.Shooter.k_ShooterArmMotorI, 
                                                                  Constants.Shooter.k_ShooterArmMotorD );
  private final DutyCycleEncoder m_ShooterArmEncoder = new DutyCycleEncoder( Constants.Shooter.k_ShooterArmEncoderId );
  //private final DigitalInput m_IntakeLimitSwitch = new DigitalInput( Constants.Intake.k_IntakeLimitSwitchId );

  
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
    m_LeftShooterArmMotor = new CANSparkMax( Constants.Shooter.k_ShooterArmLeftMotorCanId, MotorType.kBrushless );
    m_RightShooterArmMotor = new CANSparkMax( Constants.Shooter.k_ShooterArmRightMotorCanId, MotorType.kBrushless );

    /* Restore them to defaults */
    m_TopShooterMotor.restoreFactoryDefaults();
    m_BotShooterMotor.restoreFactoryDefaults();
    m_LeftShooterArmMotor.restoreFactoryDefaults();
    m_RightShooterArmMotor.restoreFactoryDefaults();

    /* Set the motors Idle Mode */
    m_TopShooterMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );
    m_BotShooterMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );
    m_LeftShooterArmMotor.setIdleMode( CANSparkBase.IdleMode.kBrake );
    m_RightShooterArmMotor.setIdleMode( CANSparkBase.IdleMode.kBrake );

    /* Set the motor Inversions */
    m_TopShooterMotor.setInverted( false );  // GTH:TODO need to update
    m_BotShooterMotor.setInverted( true ); // GTH:TODO need to update
    m_LeftShooterArmMotor.setInverted( true ); // GTH:TODO need to update
    m_RightShooterArmMotor.setInverted( true ); // GTH:TODO need to update

    /* Setup the PID Controllers */
    m_LeftShooterPID = m_TopShooterMotor.getPIDController();
    m_LeftShooterPID.setP( Constants.Shooter.k_ShooterMotorP );
    m_LeftShooterPID.setI( Constants.Shooter.k_ShooterMotorI );
    m_LeftShooterPID.setD( Constants.Shooter.k_ShooterMotorD );
    m_LeftShooterPID.setFF( Constants.Shooter.k_ShooterMotorFF );
    m_LeftShooterPID.setOutputRange( Constants.Shooter.k_ShooterMinOutput, Constants.Shooter.k_ShooterMaxOutput );

    m_RightShooterPID = m_BotShooterMotor.getPIDController();
    m_RightShooterPID.setP( Constants.Shooter.k_ShooterMotorP );
    m_RightShooterPID.setI( Constants.Shooter.k_ShooterMotorI );
    m_RightShooterPID.setD( Constants.Shooter.k_ShooterMotorD );
    m_RightShooterPID.setFF( Constants.Shooter.k_ShooterMotorFF );
    m_RightShooterPID.setOutputRange( Constants.Shooter.k_ShooterMinOutput, Constants.Shooter.k_ShooterMaxOutput );

    //m_ShooterArmPID. = 

    /* Setup the Motor Encoders */
    m_LeftShooterEncoder = m_TopShooterMotor.getEncoder();
    m_RightShooterEncoder = m_BotShooterMotor.getEncoder();
  }

  private static class PeriodicIO 
  {
    double shooter_rpm = 0.0;
    ShooterState Shooter_Target = ShooterState.NONE;

    ShooterArmState ShooterArm_Target = ShooterArmState.NONE;
    double ShooterArm_Voltage = 0.0;

  }

  /*-------------------- Generic Subsystem Functions -------------------------*/

  @Override
  public void periodic() 
  {
    /* Check to see if we have a note and need to verify position. */
    checkAutoTasks();

    /* Shooter Arm Control */
    double ShooterArm_angle = pivotTargetToAngle( m_PeriodicIO.ShooterArm_Target );
    m_PeriodicIO.ShooterArm_Voltage = m_ShooterArmPID.calculate( getPivotAngleDegrees(), ShooterArm_angle );

    /* Add a failsafe to check to see if encoder is connected. */
    if( m_ShooterArmEncoder.get() == 0.0 )
    {
    /* chances are it is not connected, so disable it. */
    m_PeriodicIO.ShooterArm_Voltage = 0.0;
    }

    /* Control the intake */
    m_PeriodicIO.shooter_rpm = intakeStateToSpeed( m_PeriodicIO.Shooter_Target );
    SmartDashboard.putString( "Shooter State:", m_PeriodicIO.Shooter_Target.toString() );
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

    SmartDashboard.putNumber("Shooter Set Speed:", intakeStateToSpeed(m_PeriodicIO.Shooter_Target));
    SmartDashboard.putNumber("Shooter Arm Abs Enc (get):", m_ShooterArmEncoder.get());
    SmartDashboard.putNumber("Shooter Arm Abs Enc (getAbsolutePosition):", m_ShooterArmEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Shooter Arm Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());
    SmartDashboard.putNumber("Shooter Arm Setpoint:", pivotTargetToAngle( m_PeriodicIO.ShooterArm_Target ));

    SmartDashboard.putNumber("Shooter Arm Power:", m_PeriodicIO.ShooterArm_Voltage);
    SmartDashboard.putNumber("Shooter Arm Left Current:", m_LeftShooterArmMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Arm Right Current:", m_RightShooterArmMotor.getOutputCurrent());

  }

  @Override
  public void reset() 
  {

  }

  /*---------------------- Custom Public Functions ---------------------------*/

  public enum ShooterArmState
  {
    NONE, 
    STOWED,
    AMP,
    SPEAKER,
    STAGE,
    PASS
  }

  public enum ShooterState
  {
    NONE,
    SPEAKER,
    AMP,
    HOLD,
    PASS,
    STAGE
  }

  public double getPivotAngleDegrees() 
  {
    double value = m_ShooterArmEncoder.getAbsolutePosition() 
                  - Constants.Shooter.k_ShooterArmEncoderOffset 
                  + 0.5;

    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public double pivotTargetToAngle( ShooterArmState state )  
  {
    switch( state ) 
    {
      case NONE:
      case STOWED:
        return Constants.Shooter.k_ShooterArmAngleStowed;
      case AMP:
        return Constants.Shooter.k_ShooterArmAngleAmp;
      case SPEAKER:
        return Constants.Shooter.k_ShooterArmAngleSpeaker;
      case STAGE:
        return Constants.Shooter.k_ShooterArmAngleStage;
      default:
        return Constants.Shooter.k_ShooterArmAngleStowed;
    }
  }

  public double intakeStateToSpeed( ShooterState state ) 
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
    m_PeriodicIO.Shooter_Target = ShooterState.NONE;
  }

  public void setShooterArmTarget( ShooterArmState target ) 
{
  m_PeriodicIO.ShooterArm_Target = target;
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

private boolean isPivotAtTarget() 
{
  return Math.abs( getPivotAngleDegrees() - pivotTargetToAngle(m_PeriodicIO.ShooterArm_Target ) ) < 5;
}

}
