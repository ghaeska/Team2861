package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase
{
    private static CANSparkMax
      m_intakeTest = new CANSparkMax(Constants.intakeCanID, MotorType.kBrushless);

    public void intakeTestMotor(double testSpeed)
    {
        m_intakeTest.set(testSpeed);

    }



}
