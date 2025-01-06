package frc.robot.subsystems.launcherAssembly.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.field.ScoringPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.Types.PositionType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterWheelSubsystem m_left_wheel;
    private ShooterWheelSubsystem m_right_wheel;

    private double forwardSpeed = 0;
    private DriveSubsystem m_drive;
    private boolean m_autoSpeed = true;

    public ShooterSubsystem(DriveSubsystem drive) {
        m_left_wheel = new ShooterWheelSubsystem(Constants.LeftShooterConstants.kPidValues,
                Constants.LeftShooterConstants.kFFValues, Constants.LeftShooterConstants.kDeviceId, "Left", false,
                Constants.ShooterConstants.kDesiredSpeed, Constants.ShooterConstants.kReverseSpeed);
        m_right_wheel = new ShooterWheelSubsystem(Constants.RightShooterConstants.kPidValues,
                Constants.RightShooterConstants.kFFValues, Constants.RightShooterConstants.kDeviceId, "Right", true,
                Constants.ShooterConstants.kDesiredSpeed, Constants.ShooterConstants.kReverseSpeed);

        forwardSpeed = 0;
        m_drive = drive;
    }

    @Override
    public void periodic() {
        if (m_autoSpeed) {
            var desiredShootingSpeed = MathUtils.distanceToShootingSpeed(m_drive.getPose());
            SmartDashboard.putBoolean("Shooter At Speed",  shooterAtSpeedOrFaster());
            if(desiredShootingSpeed != forwardSpeed){
                m_left_wheel.setNewForwardSpeed(desiredShootingSpeed);
                m_right_wheel.setNewForwardSpeed(desiredShootingSpeed);
                forwardSpeed = desiredShootingSpeed;
            }
        }
       
    }

    public Command SetAutoSpeed(){
        return Commands.runOnce(() -> m_autoSpeed = true);
    }
    public Command SetManualSpeed(){
        return Commands.runOnce(() -> m_autoSpeed = false);
    }
    /**
     * Stop the shooter, when no command is running. Note this does not work if a
     * command is currently running.
     */
    public Command StopShooterCommand() {
        return Commands.parallel(Commands.runOnce(() -> m_left_wheel.StopShooter()),
                Commands.runOnce(() -> m_right_wheel.StopShooter()));
    }

    public void StopShooter(){
        m_left_wheel.StopShooter();
        m_right_wheel.StopShooter();
    }

    public Command SpoolShooterCommand(){
         return Commands.parallel(Commands.runOnce(() -> m_left_wheel.SpoolShooter()),
                Commands.runOnce(() -> m_right_wheel.SpoolShooter()));
    }

    public Command RunShooterForwardCommand(boolean FinishWhenAtTargetSpeed) {
        return Commands.parallel(m_left_wheel.RunShooterForwardCommand(FinishWhenAtTargetSpeed),
                m_right_wheel.RunShooterForwardCommand(FinishWhenAtTargetSpeed));
    }

    public Command EjectCommand(){
        return Commands.parallel(m_left_wheel.EjectCommand(),
                m_right_wheel.EjectCommand());
    }

    public Command RunShooterForwardForAmp(){
        return Commands.parallel(m_left_wheel.RunShooterForwardForAmp(),
                m_right_wheel.RunShooterForwardForAmp());
    }

    public Command RunShooterForwardIdle(){
        return Commands.parallel(m_left_wheel.RunShooterForwardIdle(),
                m_right_wheel.RunShooterForwardIdle());
    }

    public Command RunShooterForwardForScorpion(boolean FinishWhenAtTargetSpeed){
        return Commands.parallel(m_left_wheel.RunShooterForwardForScorpion(FinishWhenAtTargetSpeed),
                m_right_wheel.RunShooterForwardForScorpion(FinishWhenAtTargetSpeed));
    }

    public Command RunShooterBackwardCommand(boolean FinishWhenAtTargetSpeed) {
        return Commands.parallel(m_left_wheel.RunShooterBackwardCommand(FinishWhenAtTargetSpeed),
                m_right_wheel.RunShooterBackwardCommand(FinishWhenAtTargetSpeed));
    }

    public boolean shooterAtSpeed(){
        return m_right_wheel.shooterAtSpeed() && m_left_wheel.shooterAtSpeed() ;
    }

     public boolean shooterAtSpeedOrFaster(){
        return (m_right_wheel.shooterAtSpeed() && m_left_wheel.shooterAtSpeed()) ||  
        (m_right_wheel.isShooterRunningFaster() && m_left_wheel.isShooterRunningFaster()) ;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, PositionType whichSide) {
        if (whichSide == PositionType.LEFT) {
            return m_left_wheel.sysIdQuasistatic(direction);
        } else {
            return m_right_wheel.sysIdQuasistatic(direction);
        }

    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, PositionType whichSide) {
        if (whichSide == PositionType.LEFT) {
            return m_left_wheel.sysIdDynamic(direction);
        } else {
            return m_right_wheel.sysIdDynamic(direction);
        }
    }
}
