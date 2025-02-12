package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax m_motor;
    
    
    public ClimbSubsystem(){
        m_motor = new SparkMax(ClimbConstants.kMotorID, MotorType.kBrushless);
        m_motor.setInverted(ClimbConstants.kMotorInverted);
        
    }





    public void setSpeed(double speed){
        m_motor.set(speed);
    }

    

public Command runClimberForward(){
    return new FunctionalCommand(
        ()->{},
         ()->setSpeed(1),
         (interrupted) -> m_motor.set(0),
         () -> false, this);
}
public Command runClimberReverse(){
    return new FunctionalCommand(
        ()->{},
         ()->setSpeed(-.5),
         (interrupted) -> m_motor.set(-1),
         () -> false, this);
}

}
