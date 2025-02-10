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

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Configs;

public class AlgaeSubsystem extends SubsystemBase
{
  //Define the Motors
  private final SparkMax m_LeftAlgaeMotor;
  private final SparkMax m_RightAlgaeMotor;

  // Define a relative encoder for algae motors
  private RelativeEncoder m_LeftAlgaeEncoder;
  private RelativeEncoder m_RightAlgaeEncoder;

  public AlgaeSubsystem()
  {
    /* Setup the two motors */
    m_LeftAlgaeMotor = new SparkMax(Constants.AlgaeConstants.k_LeftAlgaeMotorCANId, MotorType.kBrushless );
    m_RightAlgaeMotor = new SparkMax(Constants.AlgaeConstants.k_RightAlgaeMotorCANId, MotorType.kBrushless );

    /* Need to setup an encoder, dont think we will need one. */
    m_LeftAlgaeEncoder = m_LeftAlgaeMotor.getEncoder();
    m_RightAlgaeEncoder = m_RightAlgaeMotor.getEncoder();

    /* No need to setup a PID loop, shouldnt need one. */

    /* Configure the left motor. */
    m_LeftAlgaeMotor.configure
    (
      Configs.AlgaeModule.AlgaeMotorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Configure the right motor. */
    m_RightAlgaeMotor.configure
    (
      /* The right motor has to follow the left, set that up. */
      Configs.AlgaeModule.AlgaeMotorConfig.follow
      ( 
        Constants.AlgaeConstants.k_LeftAlgaeMotorCANId, 
        true
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );



  }

/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    /* Print out the Algae Encoder positions and velocities */
    SmartDashboard.putNumber("LeftAlgaeEncoder:", m_LeftAlgaeEncoder.getPosition() );
    SmartDashboard.putNumber("LeftAlgaeSpeed:", m_LeftAlgaeEncoder.getVelocity() );

    SmartDashboard.putNumber("RightAlgaeEncoder:", m_RightAlgaeEncoder.getPosition() );
    SmartDashboard.putNumber("RightAlgaeSpeed:", m_RightAlgaeEncoder.getVelocity() );

}


  /********************* Helper Functions for Algae *************************/
  public void runAlgae( double voltage )
  {
    m_LeftAlgaeMotor.set( voltage );
  }

  public double getAlgaeVelocity()
  {
    return m_LeftAlgaeEncoder.getVelocity();
  }
  
  public double getAlgaePosition()
  {
    return m_LeftAlgaeEncoder.getPosition();
  }

  public void resetAlgaePosition()
  {
    m_LeftAlgaeEncoder.setPosition( 0 );
  }

  public void stopAlgae()
  {
    m_LeftAlgaeMotor.setVoltage( 0 );;
  }


  /***************************** Commands **************************************/
  
  /* Intake Algae Command */
  public Command IntakeAlgaeForwardCommand()
  {
    return new RunCommand(()->this.runAlgae(.2), this );
  }

  /* Outtake Algae Command */
  public Command IntakeAlgaeReverseCommand()
  {
    return new RunCommand(()->this.runAlgae(-.2), this );
  }

  /* Stop Algae Command */
  public Command IntakeAlgaeStopCommand()
  {
    return new RunCommand(()->this.stopAlgae(), this );
  }




  
}
