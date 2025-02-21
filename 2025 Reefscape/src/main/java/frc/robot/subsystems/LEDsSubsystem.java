package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;

import frc.robot.Constants;


public class LEDsSubsystem 
{
  /* Create a LED class on PWM port 1. */
  private AddressableLED m_LED_1 = new AddressableLED( 0 );

  /* Create a LED buffer that consists of 60 LED's */
  private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer( 60 );

  /* Create the 3 views that we want for the robot. */
  /* View 1: elevator */
  private AddressableLEDBufferView m_ElevatorView;

  /* View 2: Left Coral */
  private AddressableLEDBufferView m_LeftCoralView;

  /* View 3: Right Coral */
  private AddressableLEDBufferView m_RightCoralView;

  /* Create Some LED patters, Green, Red, Yellow */
  LEDPattern m_RedLedPattern = LEDPattern.solid( Color.kRed );
  LEDPattern m_BluLedPattern = LEDPattern.solid( Color.kBlue );
  LEDPattern m_YelLedPattern = LEDPattern.solid( Color.kYellow );
  LEDPattern m_GreLedPattern = LEDPattern.solid( Color.kGreen );

  public LEDsSubsystem()
  {
    /* Tell the LED which buffer is its. */
    m_LED_1.setLength( m_LEDBuffer.getLength() );

    /* Set the views for the different sections */
    m_ElevatorView = m_LEDBuffer.createView( 0, 39 );

    m_LeftCoralView = m_LEDBuffer.createView( 40, 49 );
    
    m_RightCoralView = m_LEDBuffer.createView( 50, 59 );

    /* Apply one color to the whole buffer */
    m_YelLedPattern.applyTo( m_LEDBuffer );
    /* Write the buffer out to the LED String. */
    m_LED_1.setData( m_LEDBuffer );
    m_LED_1.start();
  }

/******************** Functions for the Coral ****************************/
  public void setLeftCoralRedRightCoralGreen()
  {
    m_RedLedPattern.applyTo( m_LeftCoralView );
    m_GreLedPattern.applyTo( m_RightCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }

  public void setLeftCoralGreenRightCoralRed()
  {
    m_GreLedPattern.applyTo( m_LeftCoralView );
    m_RedLedPattern.applyTo( m_RightCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }

/*********************** Functions for the All LED ****************************/
  public void SetAllRedCmd()
  {
    m_RedLedPattern.applyTo( m_ElevatorView );
    m_RedLedPattern.applyTo( m_LeftCoralView );
    m_RedLedPattern.applyTo( m_RightCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }

  public void SetAllGreenCmd()
  {
    m_GreLedPattern.applyTo( m_ElevatorView );
    m_GreLedPattern.applyTo( m_LeftCoralView );
    m_GreLedPattern.applyTo( m_RightCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }
  /***************************** LED Commands *********************************/
  public Command LED_LredRgreenCmd()
  {
    return new RunCommand( ()->setLeftCoralRedRightCoralGreen() );
  }

  public Command LED_LgreenRredCmd()
  {
    return new RunCommand( ()->setLeftCoralGreenRightCoralRed() );
  }

  public Command LED_GreenAllCmd()
  {
    return new RunCommand( () -> SetAllGreenCmd() );
  }

  public Command LED_RedAllCmd()
  {
    return new RunCommand( () -> SetAllRedCmd() );
  }




}


