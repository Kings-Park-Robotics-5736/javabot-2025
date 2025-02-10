package frc.robot.subsystems.ElevateAssembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndeffectorConstants;



import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
public class EndeffectorSubsystem extends SubsystemBase{

    private SparkMax m_motor;

    private SparkLimitSwitch forwardLimitSwitch;
    private SparkLimitSwitch reverseLimitSwitch;


    private SparkMaxConfig m_FLimitEnable;
    private SparkMaxConfig m_RLimitEnable;
    private SparkMaxConfig m_AllLimitEnable;
    private SparkMaxConfig m_NoLimitEnable;

    public EndeffectorSubsystem(){
        m_motor = new SparkMax(EndeffectorConstants.kMotorID, MotorType.kBrushless);

        
        m_FLimitEnable = new SparkMaxConfig();
        m_RLimitEnable = new SparkMaxConfig();
        m_AllLimitEnable = new SparkMaxConfig();
        m_NoLimitEnable= new SparkMaxConfig();



        forwardLimitSwitch = m_motor.getForwardLimitSwitch();
        reverseLimitSwitch = m_motor.getReverseLimitSwitch();
        
    



    }

    @Override
    public void periodic() {

        

        
        
    }


    public boolean ForwardLimitReached(){
        return m_motor.getForwardLimitSwitch().isPressed();
    }
    public boolean ReverseLimitReached(){
        return m_motor.getReverseLimitSwitch().isPressed();
    }

 public void setSpeed(double speed){
     m_motor.set(speed);
 }
 public void stopEndeffector(){
    m_motor.set(0);
}








public Command endeffector_Flimit_Enable() {
    // implicitly require `this`
    return this.runOnce(() -> m_motor.configureAsync(m_FLimitEnable, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  public Command endeffector_Rlimit_Enable() {
    // implicitly require `this`
    return this.runOnce(() -> m_motor.configureAsync(m_RLimitEnable, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  public Command endeffector_Nolimit_Enable() {
    // implicitly require `this`
    return this.runOnce(() -> m_motor.configureAsync(m_NoLimitEnable, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }




   public Command RunEndeffectorManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> System.out.println("Endeffector Manual Run"),
                () -> setSpeed(getSpeed.getAsDouble()),
                (interrupted) -> m_motor.set(0),
                () -> false, this);
    }

}
