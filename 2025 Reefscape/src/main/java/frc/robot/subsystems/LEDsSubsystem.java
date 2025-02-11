package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;


public class LEDsSubsystem 
{
  /* Create a LED class on PWM port 1. */
  private AddressableLED m_LED_1 = new AddressableLED( 1 );

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
  }

/******************* Functions for the Right Coral ****************************/
  public void setRightCoralRed()
  {
    m_RedLedPattern.applyTo( m_RightCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }

  public void setRightCoralGreen()
  {
    m_GreLedPattern.applyTo( m_RightCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }

/******************** Functions for the Left Coral ****************************/
  public void setLeftCoralRed()
  {
    m_RedLedPattern.applyTo( m_LeftCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }

  public void setLeftCoralGreen()
  {
    m_GreLedPattern.applyTo( m_LeftCoralView );
    m_LED_1.setData( m_LEDBuffer );
  }

/********************** Functions for the Elevator ****************************/
  public void setElevatorRed()
  {
    m_RedLedPattern.applyTo( m_ElevatorView );
    m_LED_1.setData( m_LEDBuffer );
  }

  public void setElevatorGreen()
  {
    m_GreLedPattern.applyTo( m_ElevatorView );
    m_LED_1.setData( m_LEDBuffer );
  }




}


