package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;

import frc.robot.Constants;


public class LEDsSubsystem extends SubsystemBase
{
  /* Create a LED class on PWM port 1. */
  private AddressableLED m_LED_Left = new AddressableLED( 0 );
  //private AddressableLED m_LED_Right = new AddressableLED( 1 );

  /* Create a LED buffer that consists of 7 each LED's */
  private AddressableLEDBuffer m_LEDBufferLeft = new AddressableLEDBuffer( 14 );
  
  //private AddressableLEDBuffer m_LEDBufferRight = new AddressableLEDBuffer( 7 );

  /* Create the 3 views that we want for the robot. */
  /* View 1: elevator */
  //private AddressableLEDBufferView m_ElevatorView;

  /* View 2: Left Coral */
  private AddressableLEDBufferView m_LeftCoralView;

  /* View 3: Right Coral */
  private AddressableLEDBufferView m_RightCoralView;

  /* Create Some LED patters, Green, Red, Yellow */
  LEDPattern m_RedLedPattern = LEDPattern.solid( Color.kGreen );
  LEDPattern m_BluLedPattern = LEDPattern.solid( Color.kBlue );
  LEDPattern m_YelLedPattern = LEDPattern.solid( Color.kYellow );
  LEDPattern m_GreLedPattern = LEDPattern.solid( Color.kRed );

  public LEDsSubsystem()
  {
    /* Tell the LED which buffer is its. */
    m_LED_Left.setLength( m_LEDBufferLeft.getLength() );
    //m_LED_Right.setLength( m_LEDBufferRight.getLength() );

    /* Set the views for the different sections */
    //m_ElevatorView = m_LEDBuffer.createView( 0, 39 );

    m_LeftCoralView = m_LEDBufferLeft.createView( 0, 6 );
    
    m_RightCoralView = m_LEDBufferLeft.createView( 7, 13 );

    /* Apply one color to the whole buffer */
    m_RedLedPattern.applyTo( m_LeftCoralView );
    m_RedLedPattern.applyTo( m_RightCoralView );
    /* Write the buffer out to the LED String. */
    m_LED_Left.setData( m_LEDBufferLeft );
    //m_LED_Right.setData( m_LEDBufferRight );
    m_LED_Left.start();
    //m_LED_Right.start();
  }

/******************** Functions for the Coral ****************************/
  public void setLeftCoralRedRightCoralGreen()
  {
    m_RedLedPattern.applyTo( m_LeftCoralView );
    m_GreLedPattern.applyTo( m_RightCoralView );
    m_LED_Left.setData( m_LEDBufferLeft );
    //m_LED_Left.
    //m_LED_Right.setData( m_LEDBufferRight );
  }

  public void setLeftCoralGreenRightCoralRed()
  {
    m_GreLedPattern.applyTo( m_LeftCoralView );
    m_RedLedPattern.applyTo( m_RightCoralView );
    m_LED_Left.setData( m_LEDBufferLeft );
    //m_LED_Right.setData( m_LEDBufferRight );
  }

/*********************** Functions for the All LED ****************************/
  public void SetAllRed()
  {
    //m_RedLedPattern.applyTo( m_ElevatorView );
    m_RedLedPattern.applyTo( m_LeftCoralView );
    m_RedLedPattern.applyTo( m_RightCoralView );
    m_LED_Left.setData( m_LEDBufferLeft );
    //m_LED_Right.setData( m_LEDBufferRight );
  }

  public void SetAllGreen()
  {
    //m_GreLedPattern.applyTo( m_ElevatorView );
    m_GreLedPattern.applyTo( m_LeftCoralView );
    m_GreLedPattern.applyTo( m_RightCoralView );
    m_LED_Left.setData( m_LEDBufferLeft );
    //m_LED_Right.setData( m_LEDBufferRight );
  }
  /***************************** LED Commands *********************************/
  public Command LED_LredRgreenCmd()
  {
    //System.out.print(" LED Command 1");
    return new RunCommand( ()->setLeftCoralRedRightCoralGreen());
  }

  public Command LED_LgreenRredCmd()
  {
    //System.out.print(" LED Command 2");
    return new RunCommand( () -> this.setLeftCoralGreenRightCoralRed(), this );
  }

  public Command LED_GreenAllCmd()
  {
    return new RunCommand( () -> this.SetAllGreen(), this );
  }

  public Command LED_RedAllCmd()
  {
    return new RunCommand( () -> this.SetAllRed(), this );
  }




}


