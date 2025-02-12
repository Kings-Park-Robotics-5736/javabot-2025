package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utils.SparkMaxUtils;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax m_motor;
    private SparkMaxConfig m_motorConfig;
    
    
    public ClimbSubsystem(){
        m_motor = new SparkMax(ClimbConstants.kMotorID, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();
        m_motorConfig.idleMode(IdleMode.kBrake);
        m_motorConfig.inverted(ClimbConstants.kMotorInverted);
        if(!SparkMaxUtils.ApplySparkMaxConfig(m_motor, m_motorConfig)){
            System.out.println("Error applying climb motor config");
        }

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
         ()->setSpeed(-.8),
         (interrupted) -> m_motor.set(0),
         () -> false, this);
}

}
