package frc.utils;

public class Conversions 
{

    public static double neoToMeters( double positionCounts,
                                      double circumfernce,
                                      double gearRatio )
    {
      return ( positionCounts * ( circumfernce / ( gearRatio * 2048.0 ) ) );
    }
    
}
