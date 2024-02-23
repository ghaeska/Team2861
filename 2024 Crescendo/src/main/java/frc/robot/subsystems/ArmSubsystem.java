package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.utils.Helpers;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase
{
  private CANSparkMax m_LeftArmMotor;
  private CANSparkMax m_RightArmMotor;

  //private SparkPIDController ArmPIDController;

  private double m_armSetpointRadians;
  private double m_ArmRPM;
  private double m_armTolerance;

  private RelativeEncoder m_LeftArmEncoder;
  private RelativeEncoder m_RightArmEncoder;
  private AbsoluteEncoder m_ArmEncoder;

  //private Timer m_Timer;
  private VoltageOut m_VoltageOutput = new VoltageOut(0.0);

  //private DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder( Arm.k_ArmEncoderId );

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
    m_LeftArmMotor.setInverted( false ); // GTH:TODO need to update
    m_RightArmMotor.setInverted( false ); // GTH:TODO need to update

    /* Set the current limits on the Motors */
    m_LeftArmMotor.setSmartCurrentLimit( 30 );
    m_RightArmMotor.setSmartCurrentLimit( 30 );

    //ArmPIDController.setFeedbackDevice(m_ArmEncoder);

    /* Setup Arm Motor Encoders that are plugged into the main controller port. */
    // m_RightArmEncoder = m_RightArmMotor.getEncoder( SparkRelativeEncoder.Type.kHallSensor, 42 );
    // m_RightArmEncoder.setPositionConversionFactor( Arm.k_ArmPositionFactor );
    // m_RightArmEncoder.setVelocityConversionFactor( Arm.k_ArmVelocityFactor );
    // m_RightArmEncoder.setPosition(0.0);

    // m_LeftArmEncoder = m_LeftArmMotor.getEncoder( SparkRelativeEncoder.Type.kHallSensor, 42 );
    // m_LeftArmEncoder.setPositionConversionFactor( Arm.k_ArmPositionFactor );
    // m_LeftArmEncoder.setVelocityConversionFactor( Arm.k_ArmVelocityFactor );
    // m_LeftArmEncoder.setPosition(0.0);
    m_LeftArmEncoder = m_LeftArmMotor.getEncoder();
    m_RightArmEncoder = m_RightArmMotor.getEncoder();

    m_ArmEncoder = m_LeftArmMotor.getAbsoluteEncoder( Type.kDutyCycle );

    // ArmPIDController.setFeedbackDevice( m_ArmEncoder );
    // ArmPIDController.setP( Arm.k_ArmMotorP );
    // ArmPIDController.setI( Arm.k_ArmMotorI );
    // ArmPIDController.setD( Arm.k_ArmMotorD );
    // ArmPIDController.setFF( Arm.k_ArmMotorFF );
    // ArmPIDController.setOutputRange( Arm.k_ArmMinOutput, Arm.k_ArmMaxOutput );
    //ArmPIDController.set

    /* Create a Leader/Follower Motor for Arm System */
    /* Right motor will follow the output of the left motor. */
    m_RightArmMotor.follow( m_LeftArmMotor );

    m_LeftArmMotor.burnFlash();
    m_RightArmMotor.burnFlash();

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

    //SmartDashboard.putNumber("Shooter Arm Abs Enc (get):", m_ArmEncoder.get());
    SmartDashboard.putNumber("Shooter Arm Abs Enc (getAbsolutePosition):", m_ArmEncoder.getPosition());
    // SmartDashboard.putNumber("Shooter Arm Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());


  }

  public void runArm( double setPointRadians )
  {
    m_armSetpointRadians = setPointRadians;
    //ArmPIDController.setReference(m_armSetpointRadians, CANSparkMax.ControlType.kPosition);
    m_LeftArmMotor.set(setPointRadians);
  }

  public void stopArm()
  {
    m_LeftArmMotor.set( 0.0 );
    //m_armSetpointRadians = setPointRadians;
    //ArmPIDController.setReference(m_armSetpointRadians, CANSparkMax.ControlType.kPosition);
  }

  

/***************************** Commands ************************************* */
  public Command runArmCommand()
  {
    return new RunCommand(()->this.runArm( 0.4 ), this );
  }

  public Command runArmRevCommand()
  {
    return new RunCommand(()->this.runArm( -0.4 ), this );
  }

  public Command stopArmCommand()
  {
    return new RunCommand(()->this.stopArm(), this );
  }








  
}
