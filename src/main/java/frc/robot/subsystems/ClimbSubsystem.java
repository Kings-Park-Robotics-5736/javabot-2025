package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utils.SparkMaxUtils;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax m_motor;
    private SparkMaxConfig m_motorConfig;
    private DutyCycleEncoder m_climbEncoder = new DutyCycleEncoder(3);
    
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

    public double getPosition(){
        return m_motor.getEncoder().getPosition();
    }

    public double getEncoderPosition(){
        return m_climbEncoder.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climb Pos", getPosition());
        SmartDashboard.putNumber("ClimbEncoder", m_climbEncoder.get());
    }



public Command runClimberForward(){
    return new FunctionalCommand(
        ()->{},
         ()->setSpeed(.8),
         (interrupted) -> m_motor.set(0),
         () -> false, this);
}
public Command runClimberReverse(){
    return new FunctionalCommand(
        ()->{},
         ()->setSpeed(-1),
         (interrupted) -> m_motor.set(0),
         () -> false, this);
}


public Command runClimberToSetpointIn(double setpoint){
    return new FunctionalCommand(
        ()->{},
         ()->{if (getEncoderPosition() > setpoint){
             setSpeed(.75);
         } else if (getEncoderPosition() < setpoint){
             setSpeed(0);
         }},
         (interrupted) -> m_motor.set(0),
         () -> false, this);
}

public Command runClimberToSetpointOut(double setpoint){
    return new FunctionalCommand(
        ()->{},
         ()->{if (getEncoderPosition() < setpoint){
             setSpeed(1);
         } else if (getEncoderPosition() > setpoint){
             setSpeed(0);
         }},
         (interrupted) -> m_motor.set(0),
         () -> false, this);
}


}