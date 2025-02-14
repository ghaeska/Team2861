package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SwerveConstants.OIConstants;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase
{
    //Define the Motors
    private final SparkMax m_LeftEleMotor;
    private final SparkMax m_RightEleMotor;    

    //Define spark pid controller.
    private SparkClosedLoopController m_LeftElePIDController;

    //Define a relative encoder for both elevator motors
    private RelativeEncoder m_LeftEleEncoder;
    private RelativeEncoder m_RightEleEncoder;

    private double m_ElevatorSetpoint;
    
  public ElevatorSubsystem()
  {
    /* Assign the CAN Id's to the motors for the elevator. */
    m_LeftEleMotor  = new SparkMax( Constants.ElevatorConstants.k_LeftElevatorMotorCANId, MotorType.kBrushless );
    m_RightEleMotor = new SparkMax( Constants.ElevatorConstants.k_RightElevatorMotorCANId, MotorType.kBrushless );

    /* Setup the Elevator Encoder. */
    m_LeftEleEncoder = m_LeftEleMotor.getEncoder();
    m_RightEleEncoder = m_RightEleMotor.getEncoder();

    /* Setup the Elevator PID Loop. */
    m_LeftElePIDController = m_LeftEleMotor.getClosedLoopController();

    /* Configure the left elevator motor from the configs. */
    m_LeftEleMotor.configure
    (
      Configs.ElevatorModule.ElevatorMotorCfg,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Configure the right elevator motor from the configs. */
    m_RightEleMotor.configure
    (
      /* The right motor has to follow the left, set that up. */
      Configs.ElevatorModule.ElevatorMotorCfg.follow
      ( 
        Constants.ElevatorConstants.k_LeftElevatorMotorCANId, 
        false
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    //m_ElevatorSetpoint = m_LeftEleEncoder.getPosition();
    //setElePosition( m_ElevatorSetpoint );
    setElePosition( 0 );
  }
/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    // if( 0 )
    // {
    //   System.out.print( " Elevator PID Set Position: " );
    //   System.out.print( m_ElevatorSetpoint );
    //   System.out.print( " \r\n" );
    // }
   
    m_LeftElePIDController.setReference( m_ElevatorSetpoint, ControlType.kPosition );

    

    /* Print out the Elevator Encoder positions. */
    SmartDashboard.putNumber("RightElevatorPosition:", m_RightEleEncoder.getPosition() );
    SmartDashboard.putNumber("LeftElevatorPosition:", m_LeftEleEncoder.getPosition() );
  }




/********************* Helper Functions for Elevator **************************/
    
  private boolean onTarget()
  {
    double error = getError();
    return Math.abs( error ) < 1 ;
  }

  private double getError()
  {
    double error = m_LeftEleEncoder.getPosition() - m_ElevatorSetpoint;
    return error;

    //return m_LeftEleEncoder.getPosition().minus( m_ElevatorSetpoint );
  }


  public void runElevator( double voltage )
  {
    if( m_LeftEleEncoder.getPosition() > Constants.ElevatorConstants.k_Ele_MaxHeight )
    {
      if( voltage > 0 )
      {
        m_LeftEleMotor.set( 0 );
      }
      else
      {
        m_LeftEleMotor.set( voltage );
      }
      
    }
    else if( m_LeftEleEncoder.getPosition() < 0 )
    {
      if( voltage < 0 )
      {
        m_LeftEleMotor.set( 0 );
      }
      else
      {
        m_LeftEleMotor.set( voltage );
      }

    }
    else
    {
       m_LeftEleMotor.set( voltage );
    }
   
  }
  public double getElevatorPosition()
  {
    return m_LeftEleEncoder.getPosition();
  }

  public double getElevatorVelocity()
  {
    return m_LeftEleEncoder.getVelocity();
  }

  public void resetElevatorPosition()
  {
    m_LeftEleEncoder.setPosition( 0 );
    m_RightEleEncoder.setPosition( 0 );
  }

  public void stopElevator()
  {
    m_LeftEleMotor.setVoltage( 0 );
  }

  private void setElePosition( double position )
  {
    //m_LeftElePIDController.setReference(position, ControlType.kPosition );
    //m_LeftEleMotor.getClosedLoopController().setReference(position, ControlType.kPosition );
    if( position < 0 )
    {
      position = 0;
    }
    else if( position > 90 )
    {
      position = 89;
    }
    
    m_ElevatorSetpoint = position;  
  }

  /***************************** Commands **************************************/
  /* Stow the Elevator Command */
  public Command ElevatorToStowCmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_StowHeight );
  }

  /* Lift to the coral Source Command. */
  public Command ElevatorToSourceCmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_SrcHeight );
  }
  /* Lift to L1 reef Command. */
  public Command ElevatorToL1Cmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_L1Height );
  }
  /* Lift to L2 reef Command. */
  public Command ElevatorToL2Cmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_L2Height );
  }
  /* Lift to L3 reef Command. */
  public Command ElevatorToL3Cmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_L3Height );
  }
  /* Lift to L4 reef Command. */
  public Command ElevatorToL4Cmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_L4Height );
  }
  /* Lift to top Algea in reef Command. */
  public Command ElevatorToTopAlgaeCmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_HighAlgaeHeight );
  }
  /* Lift to Proccessor Command. */
  public Command ElevatorToProcessorCmd()
  {
    return PositionElevatorCmd( ElevatorConstants.k_Ele_ScoreAlgaeHeight );
  }

  /* Manual Up with button Command */
  public Command ElevatorManualUp( double speed )
  {
    return new RunCommand( ()->this.runElevator( speed ), this );
  }
  /* Manual Down with Button Command */
  public Command ElevatorManualDown( double speed )
  {
    return new RunCommand( ()->this.runElevator( -speed ), this );
  }

  /* Command that sets the position of the elevator */
  private Command PositionElevatorCmd( double position )
  {
    //System.out.print( "Calling Elevator Position Command \r\n " );
    return run( () -> setElePosition(position)).until( this::onTarget );
  }

  /* Default command, not really sure if we need to make this. */
  public Command defaultCommand()
  {
    return run
    ( 
      () -> 
      {
        setElePosition( m_ElevatorSetpoint );
      }
    );
  }

  /* Manual Lifting of Elevator Command. */
  public Command ElevatorManualCmd(CommandXboxController controller )
  {
    return run
    ( () -> setElePosition
      ( 
        (-MathUtil.applyDeadband( controller.getLeftY(), OIConstants.kDriveDeadband ) * 0.5 )
      )
    );
  }

  // /* Manual Lifting of Elevator Command. */
  // public Command ElevatorManualCmd(CommandXboxController controller )
  // {
  //   return new RunCommand
  //   (
  //     () -> this.runElevator
  //     ( 
  //       -MathUtil.applyDeadband
  //       (
  //         controller.getLeftY(), 
  //         OIConstants.kDriveDeadband
  //       ) 
  //       * 1.0
  //     ), 
  //     this 
  //   );

  // }
}
