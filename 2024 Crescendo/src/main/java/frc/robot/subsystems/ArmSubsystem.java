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
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Intake.IntakeState;
import frc.utils.Helpers;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase
{
  private CANSparkMax m_LeftArmMotor;
  private CANSparkMax m_RightArmMotor;

  private SparkPIDController ArmPIDController;
  private TrapezoidProfile ArmProfile;
  private TrapezoidProfile.State m_StartArmState;
  private TrapezoidProfile.State m_EndArmState;
  private TrapezoidProfile.State m_TargetArmState;

  private double m_armSetpoint;
  private double m_ArmRPM;

  private RelativeEncoder m_LeftArmEncoder;
  private RelativeEncoder m_RightArmEncoder;

  //private Timer m_Timer;

  private final DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder( Arm.k_ArmEncoderId );

public ArmSubsystem()
{
  /********************* Setup the Motor Controllers ************************/
  /* Arm Motors */
  m_LeftArmMotor = new CANSparkMax( Arm.k_ArmLeftMotorCanId, MotorType.kBrushless );
  m_RightArmMotor = new CANSparkMax( Arm.k_ArmRightMotorCanId, MotorType.kBrushless );

  /* Restore them to defaults */
  m_LeftArmMotor.restoreFactoryDefaults();
  m_RightArmMotor.restoreFactoryDefaults();

  /* Set the motors Idle Mode */
  m_LeftArmMotor.setIdleMode( CANSparkBase.IdleMode.kBrake );
  m_RightArmMotor.setIdleMode( CANSparkBase.IdleMode.kBrake );

  /* Set the motor Inversions */
  m_LeftArmMotor.setInverted( true ); // GTH:TODO need to update
  m_RightArmMotor.setInverted( true ); // GTH:TODO need to update

  /* Set the current limits on the Motors */
  m_LeftArmMotor.setSmartCurrentLimit( 35 );
  m_RightArmMotor.setSmartCurrentLimit( 35 );

  /* Set Motor Smart Limits */
  // m_LeftArmMotor.enableSoftLimit( SoftLimitDirection.kForward, true );
  // m_LeftArmMotor.enableSoftLimit( SoftLimitDirection.kReverse, true );
  // m_RightArmMotor.enableSoftLimit( SoftLimitDirection.kForward, true );
  // m_RightArmMotor.enableSoftLimit( SoftLimitDirection.kReverse, true );

  /* Set Arm Motor Soft limits, done as in REV Robotics Code. */
  // m_LeftArmMotor.setSoftLimit( SoftLimitDirection.kForward, (float) 0);
  // m_LeftArmMotor.setSoftLimit( SoftLimitDirection.kReverse, (float) -1.5);
  // m_RightArmMotor.setSoftLimit( SoftLimitDirection.kForward, (float) 0);
  // m_RightArmMotor.setSoftLimit( SoftLimitDirection.kReverse, (float) -1.5);

  /* Setup Arm Motor Encoders as done by REV Robotics. */
  m_RightArmEncoder = m_RightArmMotor.getEncoder( SparkRelativeEncoder.Type.kHallSensor, 42 );
  m_RightArmEncoder.setPositionConversionFactor( Arm.k_ArmPositionFactor );
  m_RightArmEncoder.setVelocityConversionFactor( Arm.k_ArmVelocityFactor );
  m_RightArmEncoder.setPosition(0.0);


  ArmPIDController = m_RightArmMotor.getPIDController();
  ArmPIDController.setP( Arm.k_ArmMotorP );
  ArmPIDController.setI( Arm.k_ArmMotorI );
  ArmPIDController.setD( Arm.k_ArmMotorD );

  m_LeftArmEncoder = m_LeftArmMotor.getEncoder();
  m_RightArmEncoder = m_RightArmMotor.getEncoder();

  /* Create a Leader/Follower Motor for Arm System */
  /* Right motor will follow the output of the left motor. */
  m_RightArmMotor.follow( m_LeftArmMotor );

  m_LeftArmMotor.burnFlash();
  m_RightArmMotor.burnFlash();

  //m_armSetpoint = 

  //m_Timer.start();
  updateMotionProfile();
}

@Override
public void periodic() 
{
  SmartDashboard.putNumber("Arm speed (RPM):", m_ArmRPM );
  //SmartDashboard.getNumber("Set Shooter Speed", m_Shooter_RPM ); // See if this allows for easy speed changes.
  SmartDashboard.putNumber("Right Arm Speed:", m_RightArmEncoder.getVelocity());
  SmartDashboard.putNumber("Left Arm Speed:", m_LeftArmEncoder.getVelocity());
  //SmartDashboard.putNumber("Shooter Set Speed:", ShooterStateToSpeed( m_ShooterState ));
  SmartDashboard.putNumber("Left Shooter Arm Position:", m_LeftArmEncoder.getPosition());
  SmartDashboard.putNumber("Right Shooter Arm Position:", m_RightArmEncoder.getPosition());

  SmartDashboard.putNumber("Shooter Arm Abs Enc (get):", m_ArmEncoder.get());
  SmartDashboard.putNumber("Shooter Arm Abs Enc (getAbsolutePosition):", m_ArmEncoder.getAbsolutePosition());
   // SmartDashboard.putNumber("Shooter Arm Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());


}

public void runArm( double speed )
{
  m_ArmRPM = speed;
  m_LeftArmMotor.set(speed);
  //m_RightArmMotor.set(speed);
}

public void stopArm()
{
  m_ArmRPM = 0.0;
  m_LeftArmMotor.set( 0.0 );
  //m_RightArmMotor.set( 0.0 );
}

public void updateMotionProfile()
{
  m_StartArmState = new TrapezoidProfile.State(m_RightArmEncoder.getPosition(), m_RightArmEncoder.getVelocity());
  m_EndArmState = new TrapezoidProfile.State( m_armSetpoint, 0.0 );
  ArmProfile = new TrapezoidProfile( Arm.k_ArmMotionConstraint );
  //m_Timer.reset();
}

/***************************** Commands ************************************* */
  public Command runArmCommand()
  {
    //m_IntakeState = IntakeState.INTAKE_FAST;
    return new RunCommand(()->this.runArm( 0.4 ), this );
  }

  public Command stopArmCommand()
  {
    //m_IntakeState = IntakeState.INTAKE_FAST;
    return new RunCommand(()->this.stopArm(), this );
  }








  
}
