package frc.utils;

public class Helpers 
{
  public static double modRotations( double input ) 
  {
    input %= 1.0;
    if( input < 0.0 ) 
    {
      input += 1.0;
    }
    return input;
  }

  public static double modDegrees( double input ) 
  {
    input %= 360.0;
    if( input < 0.0 ) 
    {
      input += 360.0;
    }
    return input;
  }

  public static double clamp( double val, double min, double max )
  {
    return Math.max( min, Math.min( max, val ) );
  }
}
