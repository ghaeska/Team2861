package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase
{
  private CANSparkMax m_LeftArmMotor;
  private CANSparkMax m_RightArmMotor;

  private SparkPIDController ArmPIDController;

  private RelativeEncoder m_LeftArmEncoder;
  private RelativeEncoder m_RightArmEncoder;
  private SparkAbsoluteEncoder m_ArmEncoder;

  private Rotation2d m_ArmSetpoint = new Rotation2d();

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

    /* Set Periodic Frame limits */
    m_RightArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_LeftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    ArmPIDController = m_LeftArmMotor.getPIDController();

    m_LeftArmEncoder = m_LeftArmMotor.getEncoder();
    m_RightArmEncoder = m_RightArmMotor.getEncoder();

    m_ArmEncoder = m_LeftArmMotor.getAbsoluteEncoder( Type.kDutyCycle );
    m_ArmEncoder.setInverted( true ); //TODO: Do we need this?
    m_ArmEncoder.setPositionConversionFactor( 360 );

    ArmPIDController.setFeedbackDevice( m_ArmEncoder );
    
    ArmPIDController.setP( Arm.k_ArmMotorP );
    ArmPIDController.setI( Arm.k_ArmMotorI );
    ArmPIDController.setD( Arm.k_ArmMotorD );
    ArmPIDController.setFF( Arm.k_ArmMotorFF );
    ArmPIDController.setOutputRange( Arm.k_ArmMinOutput, Arm.k_ArmMaxOutput );
    ArmPIDController.setPositionPIDWrappingEnabled(true);
    ArmPIDController.setPositionPIDWrappingMinInput(0.0);
    ArmPIDController.setPositionPIDWrappingMaxInput(360);

    /* Create a Leader/Follower Motor for Arm System */
    /* Right motor will follow the output of the left motor. */
    m_RightArmMotor.follow( m_LeftArmMotor );

    m_LeftArmMotor.burnFlash();
    m_RightArmMotor.burnFlash();

    m_ArmSetpoint = getArmAngle();
    setArmSetpoint( m_ArmSetpoint );

  }

  public void setArmSetpoint( Rotation2d setpoint_radians ) 
  {        
    if( setpoint_radians.getDegrees() < 250 ) 
    {
      // System.out.print("Arm Set Reference: ");
      // System.out.print( setpoint_radians.getDegrees() );
      // System.out.print( " \r\n" );
      setpoint_radians = Rotation2d.fromDegrees( 270 );
    } 
    else if( setpoint_radians.getDegrees() > 350 )
    {
      setpoint_radians = Rotation2d.fromDegrees(340);
    }

    System.out.print("Arm Set Reference: ");
    System.out.print( setpoint_radians.getDegrees() );
    System.out.print( " \r\n" );

    m_ArmSetpoint = setpoint_radians;
    SmartDashboard.putNumber( " SetArmSetpoint value", setpoint_radians.getDegrees() );
    SmartDashboard.putNumber( " Global Setpoint", m_ArmSetpoint.getDegrees() );
    
  }

  public Rotation2d getArmAngle() 
  {
    //SmartDashboard.putNumber( "GetArmAngle Value",  Rotation2d.fromDegrees( m_ArmEncoder.getPosition() );
    //double temp = (double)Rotation2d.fromDegrees( m_ArmEncoder.getPosition() );
    
    return Rotation2d.fromDegrees( m_ArmEncoder.getPosition() );
  }

  

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Right Arm Speed:", m_RightArmEncoder.getVelocity());
    SmartDashboard.putNumber("Left Arm Speed:", m_LeftArmEncoder.getVelocity());
    SmartDashboard.putNumber("Left Arm Position:", m_LeftArmEncoder.getPosition());
    SmartDashboard.putNumber("Right Arm Position:", m_RightArmEncoder.getPosition());

    //SmartDashboard.putNumber("Shooter Arm Abs Enc (get):", m_ArmEncoder.get());
    SmartDashboard.putNumber("Arm Abs Enc (getAbsolutePosition):", m_ArmEncoder.getPosition());
    //SmartDashboard.putNumber("Arm Abs Enc (Degrees):", ConvertRadiansToDegrees(m_ArmEncoder.getPosition()) );
    
    System.out.print("Arm PID Set Reference: ");
    System.out.print( m_ArmSetpoint.getDegrees() );
    System.out.print( " \r\n" );
    ArmPIDController.setReference(m_ArmSetpoint.getDegrees(), ControlType.kPosition);

  }

  public double ConvertRadiansToDegrees( double Radians )
  {
    return ( Radians / Math.PI ) * 180;
  }

  // Converts degrees to Radians
  public double degreesToRadians(double degrees) 
  {
    return (degrees * Math.PI) / 180.0;
  }

  public void runArm( double setPointRadians )
  {
    //m_armSetpointRadians = setPointRadians;
    //ArmPIDController.setReference(m_armSetpointRadians, CANSparkMax.ControlType.kPosition);
    m_LeftArmMotor.set(setPointRadians);
  }

  public void stopArm()
  {
    m_LeftArmMotor.set( 0.0 );
    //m_armSetpointRadians = setPointRadians;
    //ArmPIDController.setReference(m_armSetpointRadians, CANSparkMax.ControlType.kPosition);
  }

  private boolean onTarget()
  {
    return Math.abs( getError().getDegrees() ) < 2;
  }

  private Rotation2d getError()
  {
    return getArmAngle().minus( m_ArmSetpoint);
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

  public Command StowArmCommand()
  {
    return positionArmCommand( Constants.Arm.k_ArmAngleStowed );
  }

  public Command AmpArmCommand()
  {
    return positionArmCommand( Constants.Arm.k_ArmAngleAmp );
  }

  private Command positionArmCommand( Rotation2d position )
  {
    //SmartDashboard.putNumber( " Command Position ", position.getDegrees() );
    System.out.print("Calling the positionArmCommand \r\n");
    return run( () -> setArmSetpoint(position)).until(this::onTarget);
  }

  public Command defaultCommand(Supplier<Double> armChange) 
  {
    return run(() -> {
           setArmSetpoint( m_ArmSetpoint );
        });
  }





  
}
