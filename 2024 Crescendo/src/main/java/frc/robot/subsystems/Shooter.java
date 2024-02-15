package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utils.Helpers;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends Subsystem
{
  /*-------------------- Private Instance Variables --------------------------*/
  private static Shooter m_Instance;
  private PeriodicIO m_PeriodicIO;

  private CANSparkFlex m_TopShooterMotor;
  private CANSparkFlex m_BotShooterMotor;

  private CANSparkMax m_LeftShooterArmMotor;
  private CANSparkMax m_RightShooterArmMotor;

  private BangBangController BBController;

  private SparkPIDController ArmPIDController;
  private TrapezoidProfile ArmProfile;
  private TrapezoidProfile.State m_StartArmState;
  private TrapezoidProfile.State m_EndArmState;
  private TrapezoidProfile.State m_TargetArmState;



  private RelativeEncoder m_TopShooterEncoder;
  private RelativeEncoder m_BottomShooterEncoder;

  private RelativeEncoder m_LeftShooterArmEncoder;
  private RelativeEncoder m_RightShooterArmEncoder;

  private Timer m_Timer;

  

  //private SlewRateLimiter m_ShooterSlewLimiter = new SlewRateLimiter( 1000 );
  //private SlewRateLimiter m_ShooterArmSlewLimiter = new SlewRateLimiter( 1000 );


  //private final PIDController m_ShooterArmPID = new PIDController( Constants.Shooter.k_ShooterArmMotorP,
  //                                                                 Constants.Shooter.k_ShooterArmMotorI, 
  //                                                                 Constants.Shooter.k_ShooterArmMotorD );
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

    m_Timer = new Timer();

    /********************* Setup the Motor Controllers ************************/
    /* Shooter Motors */
    m_TopShooterMotor = new CANSparkFlex( Constants.Shooter.k_ShooterTopMotorCanId, MotorType.kBrushless );
    m_BotShooterMotor = new CANSparkFlex( Constants.Shooter.k_ShooterBotMotorCanId, MotorType.kBrushless );

    /* Shooter Arm Motors */
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
    m_TopShooterMotor.setInverted( true );
    m_BotShooterMotor.setInverted( false );
    m_LeftShooterArmMotor.setInverted( true ); // GTH:TODO need to update
    m_RightShooterArmMotor.setInverted( true ); // GTH:TODO need to update

    /* Set the current limits on the Motors */
    m_BotShooterMotor.setSmartCurrentLimit( 40 );
    m_TopShooterMotor.setSmartCurrentLimit( 40 );
    m_LeftShooterArmMotor.setSmartCurrentLimit( 35 );
    m_RightShooterArmMotor.setSmartCurrentLimit( 35 );

    /* Set Motor Smart Limits */
    m_LeftShooterArmMotor.enableSoftLimit( SoftLimitDirection.kForward, true );
    m_LeftShooterArmMotor.enableSoftLimit( SoftLimitDirection.kReverse, true );
    m_RightShooterArmMotor.enableSoftLimit( SoftLimitDirection.kForward, true );
    m_RightShooterArmMotor.enableSoftLimit( SoftLimitDirection.kReverse, true );

    /* Set Arm Motor Soft limits, done as in REV Robotics Code. */
    m_LeftShooterArmMotor.setSoftLimit( SoftLimitDirection.kForward, (float) 0);
    m_LeftShooterArmMotor.setSoftLimit( SoftLimitDirection.kReverse, (float) -1.5);
    m_RightShooterArmMotor.setSoftLimit( SoftLimitDirection.kForward, (float) 0);
    m_RightShooterArmMotor.setSoftLimit( SoftLimitDirection.kReverse, (float) -1.5);

    /* Setup Arm Motor Encoders as done by REV Robotics. */
    m_RightShooterArmEncoder = m_RightShooterArmMotor.getEncoder( SparkRelativeEncoder.Type.kHallSensor, 42 );
    m_RightShooterArmEncoder.setPositionConversionFactor( Constants.Arm.k_ArmPositionFactor );
    m_RightShooterArmEncoder.setVelocityConversionFactor( Constants.Arm.k_ArmVelocityFactor );
    m_RightShooterArmEncoder.setPosition(0.0);

    //m_LeftShooterArmEncoder = m_LeftShooterArmMotor.getEncoder( SparkRelativeEncoder.Type.kHallSensor, 42 );
    //m_LeftShooterArmEncoder.setPositionConversionFactor( Constants.Arm.k_ArmPositionFactor );
    //m_LeftShooterArmEncoder.setVelocityConversionFactor( Constants.Arm.k_ArmVelocityFactor );
    //m_LeftShooterArmEncoder.setPosition(0.0);

    ArmPIDController = m_RightShooterArmMotor.getPIDController();
    ArmPIDController.setP( Constants.Arm.k_ShooterArmMotorP );
    ArmPIDController.setI( Constants.Arm.k_ShooterArmMotorI );
    ArmPIDController.setD( Constants.Arm.k_ShooterArmMotorD );

    /* Create a Bang Bang controller for the shooter. */
    BBController = new BangBangController();   

    /* Setup the Motor Encoders */
    m_TopShooterEncoder = m_TopShooterMotor.getEncoder();
    m_BottomShooterEncoder = m_BotShooterMotor.getEncoder();

    m_LeftShooterArmEncoder = m_LeftShooterArmMotor.getEncoder();
    m_RightShooterArmEncoder = m_RightShooterArmMotor.getEncoder();
    

    /* Create a Leader/Follower Motor for Arm System */
    /* Right motor will follow the output of the left motor. */
    m_RightShooterArmMotor.follow( m_LeftShooterArmMotor );
    m_BotShooterMotor.follow(m_TopShooterMotor,true);

    m_BotShooterMotor.burnFlash();
    m_TopShooterMotor.burnFlash();
    m_LeftShooterArmMotor.burnFlash();
    m_RightShooterArmMotor.burnFlash();

    
    m_Timer.start();
    updateMotionProfile();
  }

  private static class PeriodicIO 
  {
    double shooter_rpm = 0.0;
    double m_ArmSetpoint = 0.0;
    double m_ArmFeedForward = 0.0;

    ShooterState shootState = ShooterState.NONE;

    //ShooterArmState ArmSetpoint = ShooterArmState.STOWED;
    //double ShooterArm_Voltage = 0.0;

  }

  /*-------------------- Generic Subsystem Functions -------------------------*/

  @Override
  public void periodic() 
  {
    /* Check to see if we have a note and need to verify position. */
    checkAutoTasks();

    /* Check to see if our encoder is connected, if not, its not safe to to spin. */
    if( m_ShooterArmEncoder.get() == 0.0 )
    {
    /* chances are it is not connected, so disable it. */
    //m_PeriodicIO.ShooterArm_Voltage = 0.0;
    }
    
    /* Convert the set state to be a speed and angle. */
    /* Convert the set state to speed. */
    m_PeriodicIO.shooter_rpm = ShooterArmStateToSpeed( m_PeriodicIO.shootState );

    /* Convert the set state to desired angle. */





    // /* Shooter Arm Control */
    // double ShooterArm_angle = pivotTargetToAngle( m_PeriodicIO.ShooterArm_Target );

    // /* Add a failsafe to check to see if encoder is connected. */
    //  if( m_ShooterArmEncoder.get() == 0.0 )
    //  {
    //  /* chances are it is not connected, so disable it. */
    //  m_PeriodicIO.ShooterArm_Voltage = 0.0;
    //  }

    // /* Control the intake */
    // //m_PeriodicIO.shooter_rpm = ShooterArmStateToSpeed( m_PeriodicIO.Shooter_Target );
    // //SmartDashboard.putString( "Shooter State:", m_PeriodicIO.Shooter_Target.toString() );
  }

  @Override
  public void writePeriodicOutputs() 
  {
    /**************** Set the control of our shooter motors. ******************/
    /* 
    ** Because our bottom motor is set to follow the top motor, we should only
    ** need to set the speed of our top motor. 
    */
    m_TopShooterMotor.set( BBController.calculate (m_TopShooterEncoder.getVelocity(), m_PeriodicIO.shooter_rpm ));


    /**************** Setup our periodic control of the arm. ******************/
    double elapsedTime = m_Timer.get();

    if( ArmProfile.isFinished( elapsedTime ) )
    {
      m_TargetArmState = new TrapezoidProfile.State( m_PeriodicIO.m_ArmSetpoint, 0.0 );
    }
    else
    {
      m_TargetArmState = ArmProfile.calculate(elapsedTime, m_StartArmState, m_EndArmState );
    }

    m_PeriodicIO.m_ArmFeedForward = Constants.Arm.k_ArmFeedForward.calculate( m_RightShooterArmEncoder.getPosition() 
                                                                              + Constants.Arm.k_ArmZeroCosineOffset, 
                                                                              m_TargetArmState.velocity );                         

    ArmPIDController.setReference( m_TargetArmState.position, CANSparkMax.ControlType.kPosition, 0, m_PeriodicIO.m_ArmFeedForward );



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

    SmartDashboard.putNumber("Shooter Top speed:", m_TopShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Bottom speed:", m_BottomShooterEncoder.getVelocity());
    
    SmartDashboard.putNumber("Left Shooter Arm Position:", m_LeftShooterArmEncoder.getPosition());
    SmartDashboard.putNumber("Right Shooter Arm Position:", m_RightShooterArmEncoder.getPosition());

    //SmartDashboard.putNumber("Shooter Set Speed:", ShooterArmStateToSpeed(m_PeriodicIO.Shooter_Target));
    SmartDashboard.putNumber("Shooter Arm Abs Enc (get):", m_ShooterArmEncoder.get());
    SmartDashboard.putNumber("Shooter Arm Abs Enc (getAbsolutePosition):", m_ShooterArmEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Shooter Arm Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());
    //SmartDashboard.putNumber("Shooter Arm Setpoint:", pivotTargetToAngle( m_PeriodicIO.ShooterArm_Target ));

    //SmartDashboard.putNumber("Shooter Arm Power:", m_PeriodicIO.ShooterArm_Voltage);
    SmartDashboard.putNumber("Shooter Arm Left Current:", m_LeftShooterArmMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Arm Right Current:", m_RightShooterArmMotor.getOutputCurrent());

  }

  @Override
  public void reset() 
  {

  }

  /*---------------------- Custom Public Functions ---------------------------*/

  public void updateMotionProfile()
  {
    m_StartArmState = new TrapezoidProfile.State(m_RightShooterArmEncoder.getPosition(), m_RightShooterArmEncoder.getVelocity());
    m_EndArmState = new TrapezoidProfile.State(m_PeriodicIO.m_ArmSetpoint, 0.0);
    ArmProfile = new TrapezoidProfile(Constants.Arm.k_ArmMotionConstraint);
    m_Timer.reset();
  }

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

  public double ShooterArmStateToSpeed( ShooterState state ) 
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
    //m_PeriodicIO.Shooter_Target = ShooterState.NONE;
  }

  public void setShooterTarget( ShooterState target ) 
  {
    m_PeriodicIO.shootState = target;
  }

  //public void setShooterTarget( Shooter)

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

// private boolean isPivotAtTarget() 
// {
//   //return Math.abs( getPivotAngleDegrees() - pivotTargetToAngle(m_PeriodicIO.ShooterArm_Target ) ) < 5;
// }

}
