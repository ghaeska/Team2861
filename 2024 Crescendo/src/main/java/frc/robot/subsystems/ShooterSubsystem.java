package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase
{
  /*-------------------- Private Instance Variables --------------------------*/
  //private static Shooter m_Instance;
  //private PeriodicIO m_PeriodicIO;

  private CANSparkFlex m_TopShooterMotor;
  private CANSparkFlex m_BotShooterMotor;

  private SparkPIDController m_TopShooterPID;
  private SparkPIDController m_BottomShooterPID;

  private RelativeEncoder m_TopShooterEncoder;
  private RelativeEncoder m_BottomShooterEncoder;

  private BangBangController BBController;

  private double m_Shooter_RPM;
  
  public ShooterSubsystem()
  {
    /* Setup the Motor Controllers */
    m_TopShooterMotor = new CANSparkFlex( Shooter.k_ShooterTopMotorCanId, MotorType.kBrushless );
    m_BotShooterMotor = new CANSparkFlex( Shooter.k_ShooterBotMotorCanId, MotorType.kBrushless );

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
    //BBController = new BangBangController();

    /* Set the PID controller for the motors */
    m_TopShooterPID = m_TopShooterMotor.getPIDController();
    m_TopShooterPID.setP( Constants.Shooter.k_ShooterMotorP );
    m_TopShooterPID.setI( Constants.Shooter.k_ShooterMotorI );
    m_TopShooterPID.setD( Constants.Shooter.k_ShooterMotorD );
    m_TopShooterPID.setFF( Constants.Shooter.k_ShooterMotorFF );
    m_TopShooterPID.setOutputRange( Constants.Shooter.k_ShooterMinOutput, Constants.Shooter.k_ShooterMaxOutput );

    m_BottomShooterPID = m_BotShooterMotor.getPIDController();
    m_BottomShooterPID.setP( Constants.Shooter.k_ShooterMotorP );
    m_BottomShooterPID.setI( Constants.Shooter.k_ShooterMotorI );
    m_BottomShooterPID.setD( Constants.Shooter.k_ShooterMotorD );
    m_BottomShooterPID.setFF( Constants.Shooter.k_ShooterMotorFF );
    m_BottomShooterPID.setOutputRange( Constants.Shooter.k_ShooterMinOutput, Constants.Shooter.k_ShooterMaxOutput );

    /* Setup the Motor Encoders */
    m_TopShooterEncoder = m_TopShooterMotor.getEncoder();
    m_BottomShooterEncoder = m_BotShooterMotor.getEncoder();

    /* Create a Leader/Follower Motor for Shooter System */
    //m_BotShooterMotor.follow(m_TopShooterMotor,true);

    /* Burn the new configurations into the motor controllers flash. */
    m_BotShooterMotor.burnFlash();
    m_TopShooterMotor.burnFlash();
  }

  public void runShooter( double setPoint_RPM )
  {
    m_Shooter_RPM = setPoint_RPM;
    m_TopShooterPID.setReference( setPoint_RPM, CANSparkBase.ControlType.kVelocity);
    m_BottomShooterPID.setReference(setPoint_RPM, CANSparkBase.ControlType.kVelocity);
  }

  public void stopShooter()
  {
    m_Shooter_RPM = 0.0;
    m_TopShooterMotor.set( m_Shooter_RPM );
    m_BotShooterMotor.set( m_Shooter_RPM );
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber( "Shooter Setpoint speed (RPM):", m_Shooter_RPM );
    SmartDashboard.putNumber( "Shooter Top Motor speed:", m_TopShooterEncoder.getVelocity() );
    SmartDashboard.putNumber( "Shooter Bottom Motor speed:", m_BottomShooterEncoder.getVelocity() );
    SmartDashboard.putNumber( "Shooter Top Motor Current", m_TopShooterMotor.getOutputCurrent() );
    SmartDashboard.putNumber( "Shooter Bottom Motor Current", m_BotShooterMotor.getOutputCurrent() );
  }


/***************************** Commands ************************************* */
  public Command runShooterSpeakerCommand()
  {
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Speaker ), this );
  }

  public Command runShooterStopCommand()
  {
    return new RunCommand(()->this.stopShooter(), this );
  }

  public Command runShooterAmpCommand()
  {
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Amp ), this );
  }

  public Command runShooterPassCommand()
  {
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Pass ), this );
  }

  public Command runShooterStageCommand()
  {
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Stage ), this );
  }















}
