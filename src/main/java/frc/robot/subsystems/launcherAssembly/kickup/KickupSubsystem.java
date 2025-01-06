package frc.robot.subsystems.launcherAssembly.kickup;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class KickupSubsystem extends SubsystemBase {
    
    private KickupWheelSubsystem m_right_wheel;

    public KickupSubsystem() {
        m_right_wheel = new KickupWheelSubsystem(Constants.RightKickupConstants.kPidValues,
        Constants.RightKickupConstants.kFFValues, Constants.RightKickupConstants.kDeviceId, "Right",true);
    
    }

    @Override
    public void periodic() {
        //leave blank
    }

    public Command RunKickupForwardCommand() {
        return m_right_wheel.RunKickupForwardCommand();
    }

    public Command RunKickupBackwardCommand() {
        return m_right_wheel.RunKickupBackwardCommand();
    }

    public Command RunKickupHoldCommand(){
        return Commands.runOnce(()->m_right_wheel.RunKickupHold());
    }
    public void RunKickupHold(){
        m_right_wheel.RunKickupHold();
    }

     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_right_wheel.sysIdQuasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return m_right_wheel.sysIdDynamic(direction);
    }
}
