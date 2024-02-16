package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class ArmSubsystem extends ProfiledPIDSubsystem
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
  private double m_armTolerance;

  private RelativeEncoder m_LeftArmEncoder;
  private RelativeEncoder m_RightArmEncoder;

  //private Timer m_Timer;
  private VoltageOut m_VoltageOutput = new VoltageOut(0.0);

  private final DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder( Arm.k_ArmEncoderId );

  public ArmSubsystem()
  {
    /* Setup the Profiled PID Controller */
    super(
          new ProfiledPIDController( Arm.k_ArmMotorP, 
                                     Arm.k_ArmMotorI,
                                     Arm.k_ArmMotorD,
                                     new TrapezoidProfile.Constraints(Arm.k_ArmCruise, Arm.k_ArmAccel ) 
                                   ) 
        );

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

  /**
   * Consumes the output from the ProfiledPIDController.
   * 
   * The PIDSubsystem will automatically call this method from its periodic()
   * block, and pass it the computed output of the control loop.
   * 
   * @param output   the output of the ProfiledPIDController
   * @param setpoint the setpoint state of the ProfiledPIDController
   */
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) 
  {
    // Correct the passed-in current setpoint before calculating the feedforward
    double correctedPosition = correctArmJointRadiansForFeedFwd(setpoint.position);

    // Calculate the feedforward using the corrected setpoint
    double feedforward = Arm.k_ArmFeedForward.calculate(correctedPosition, setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    //m_armLeader.setControl(m_VoltageOutput.withOutput(output + feedforward));
    m_LeftArmMotor.set( output + feedforward ); //I think this is right?
  }

  // Takes a position in radians relative to STOWED, and corrects it to be
  // relative to a HORIZONTAL position of zero.
  // This is used for Feedforward only, where we account for gravity using a
  // cosine function
  public double correctArmJointRadiansForFeedFwd(double position) 
  {
    return position - degreesToRadians(ArmConstants.kARM_HORIZONTAL_OFFSET - ArmConstants.kARM_STARTING_OFFSET);
  }

  /**
   * Returns the measurement of the process variable used by the
   * ProfiledPIDController.
   * 
   * The PIDSubsystem will automatically call this method from its periodic()
   * block, and pass the returned value to the control loop.
   * 
   * @return the measurement of the process variable, in this case, the Arm angle,
   *         in radians corrected to 0.0 at the STOWED position
   */
  @Override
  public double getMeasurement() 
  {
      return getArmJointRadians();
  }

  /** Override the enable() method so we can set the goal to the current position
   * 
   *  The super method resets the controller and sets its current setpoint to the 
   *  current position, but it does not reset the goal, which will cause the Arm
   *  to jump from the current position to the old goal. 
   */
  @Override
  public void enable() 
  {
    super.enable();
    m_armSetpoint = getArmJointDegrees(); 
    setGoal(getArmJointRadians());
  }

  // Converts the current encoder reading to Degrees, and corrects relative to a
  // STOWED position of zero.
    public double getArmJointDegrees() 
    {
      return dutyCycleToDegrees(getJointPosAbsolute()) - Arm.k_ArmEncoderOffset;
  }

  // Converts DutyCycle units to Degrees
  public double dutyCycleToDegrees(double dutyCyclePos) 
  {
    return dutyCyclePos * 360;
  }

  // Converts the current encoder reading to Degrees, and corrects relative to a
  // STOWED position of zero.
  public double getArmJointRadians() 
  {
    return dutyCycleToRadians(getJointPosAbsolute()) - degreesToRadians(Arm.k_ArmEncoderOffset);
  }

  // Returns the current encoder absolute value in DutyCycle units (~0 -> ~1)
  public double getJointPosAbsolute() 
  {
    return m_ArmEncoder.getAbsolutePosition();
  }

  // Converts degrees to Radians
  public double degreesToRadians(double degrees) 
  {
    return (degrees * Math.PI) / 180.0;
  }

  // Converts DutyCycle units to Radians
  public double dutyCycleToRadians(double dutyCyclePos) 
  {
    return dutyCyclePos * 2.0 * Math.PI;
  }

  // Get the current Arm Joint position error (in degrees)
  public double getArmJointError() 
  {
    return Math.abs(m_armSetpoint - getArmJointDegrees());
  }

  // Check if Arm is at the setpoint (or within tolerance)
  public boolean isArmJointAtSetpoint() 
  {
    return getArmJointError() < m_armTolerance;
  }

  // Drive the Arm directly by providing a supply voltage value
  public void setArmVoltage(double voltage)
  {
    //m_armLeader.setControl(m_VoltageOutput.withOutput(voltage));
    m_LeftArmMotor.set( voltage );
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
