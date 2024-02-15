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
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase
{
  /*-------------------- Private Instance Variables --------------------------*/
  //private static Shooter m_Instance;
  //private PeriodicIO m_PeriodicIO;

  private CANSparkFlex m_TopShooterMotor;
  private CANSparkFlex m_BotShooterMotor;

  //private SparkPIDController m_TopShooterPID;
  //private SparkPIDController m_BottomShooterPID;

  private RelativeEncoder m_TopShooterEncoder;
  private RelativeEncoder m_BottomShooterEncoder;

  private BangBangController BBController;

  private double m_Shooter_RPM;
  private ShooterState m_ShooterState;

  
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

  public void runShooter( double speed )
  {
    m_Shooter_RPM = speed;

    m_TopShooterMotor.set( BBController.calculate (m_TopShooterEncoder.getVelocity(), m_Shooter_RPM ));

  }

  public void stopShooter()
  {
    m_Shooter_RPM = 0.0;
    m_ShooterState = ShooterState.STOP;
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Shooter speed (RPM):", m_Shooter_RPM );
    SmartDashboard.getNumber("Set Shooter Speed", m_Shooter_RPM ); // See if this allows for easy speed changes.
    SmartDashboard.putNumber("Shooter Top speed:", m_TopShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Bottom speed:", m_BottomShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Set Speed:", ShooterStateToSpeed( m_ShooterState ));
  }

  public enum ShooterState
  {
    STOP,
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
      case STOP:
        return 0.0;
      case SPEAKER:
        return Shooter.k_ShooterSpeed_Speaker;
      case AMP:
        return Shooter.k_ShooterSpeed_Amp;
      case HOLD:
        return 0.0;
      case PASS:
        return Shooter.k_ShooterSpeed_Pass;
      case STAGE:
        return Shooter.k_ShooterSpeed_Stage;
      default:
        // "Safe" default
        return 0.0;
    }
  }

/***************************** Commands ************************************* */
  public Command runShooterSpeakerCommand()
  {
    m_ShooterState = ShooterState.SPEAKER;
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Speaker ), this );
  }

  public Command runShooterStopCommand()
  {
    m_ShooterState = ShooterState.STOP;
    return new RunCommand(()->this.stopShooter(), this );
  }

  public Command runShooterAmpCommand()
  {
    m_ShooterState = ShooterState.AMP;
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Amp ), this );
  }

  public Command runShooterPassCommand()
  {
    m_ShooterState = ShooterState.PASS;
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Pass ), this );
  }

  public Command runShooterStageCommand()
  {
    m_ShooterState = ShooterState.STAGE;
    return new RunCommand(()->this.runShooter( Shooter.k_ShooterSpeed_Stage ), this );
  }















}
