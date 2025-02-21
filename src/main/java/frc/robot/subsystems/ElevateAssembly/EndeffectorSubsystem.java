package frc.robot.subsystems.ElevateAssembly;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndeffectorConstants;



import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.AbsoluteEncoder;

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
    private AbsoluteEncoder m_AbsoluteEncoder;

    public EndeffectorSubsystem(){
        m_motor = new SparkMax(EndeffectorConstants.kMotorID, MotorType.kBrushless);

        
        m_FLimitEnable = new SparkMaxConfig();
        m_RLimitEnable = new SparkMaxConfig();
        m_AllLimitEnable = new SparkMaxConfig();
        m_NoLimitEnable= new SparkMaxConfig();



        forwardLimitSwitch = m_motor.getForwardLimitSwitch();
        reverseLimitSwitch = m_motor.getReverseLimitSwitch();
        
        m_AbsoluteEncoder = m_motor.getAbsoluteEncoder();




    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Arm Encoder Position", getArmEncoder());

        SmartDashboard.putNumber("Arm Encoder RAW Position", m_AbsoluteEncoder.getPosition());

        SmartDashboard.putBoolean("Forward Limit Reached", ForwardLimitReached());
        SmartDashboard.putBoolean("Reverse Limit Reached", ReverseLimitReached());



        
        
    }


    public boolean ForwardLimitReached(){
        return m_motor.getForwardLimitSwitch().isPressed();
    }
    public boolean ReverseLimitReached(){
        return m_motor.getReverseLimitSwitch().isPressed();
    }

 public void setSpeed(double speed){
    SmartDashboard.putNumber("Endeffector Speed", speed);
     m_motor.set(speed);
 }
 public void stopEndeffector(){
    m_motor.set(0);
}





public double getArmEncoder(){
    return (1 - m_AbsoluteEncoder.getPosition()) - ArmConstants.kAbsoluteOffset;
}




  public Command ReGrip(){
    return (new FunctionalCommand(
        ()->{},
         ()->setSpeed(.4),
         (interrupted) -> m_motor.set(0),
            () -> ReverseLimitReached(), this).andThen(
                new FunctionalCommand(
                    ()->{},
                     ()->setSpeed(-.2),
                     (interrupted) -> {m_motor.set(0); System.out.println("Finished Regrip");},
                        () -> !ReverseLimitReached(), this)
            )).raceWith(new WaitCommand(.75));
  }

  public Command ReGripL2L3(){
    return (new FunctionalCommand(
        ()->{},
         ()->setSpeed(.4),
         (interrupted) -> m_motor.set(0),
            () -> ReverseLimitReached(), this).andThen(
                new FunctionalCommand(
                    ()->{},
                     ()->setSpeed(-.2),
                     (interrupted) -> m_motor.set(0),
                        () -> !ReverseLimitReached(), this)
                        .andThen(new InstantCommand(()->setSpeed(.2)))
                        .andThen(new WaitCommand(.1))
                        .andThen(new InstantCommand(()->m_motor.set(0)))
            )).raceWith(new WaitCommand(.75));
  }

  //positive speed brings up, revese limit switch is the top limit

  public Command Intake(){
    return new FunctionalCommand(
        ()->{},
         ()->setSpeed(.5 ),
         (interrupted) -> m_motor.set(0),
            () -> ReverseLimitReached(), this);
  }

  public Command Score(Boolean l4Score){
    return new FunctionalCommand(
        ()->{},
         ()->setSpeed(l4Score ? 1 : -1),
         (interrupted) -> m_motor.set(0),
            () -> !ReverseLimitReached() && !ForwardLimitReached(), this);
  }

   public Command RunEndeffectorManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> System.out.println("Endeffector Manual Run"),
                () -> setSpeed(getSpeed.getAsDouble()),
                (interrupted) -> m_motor.set(0),
                () -> false, this);
    }

}
