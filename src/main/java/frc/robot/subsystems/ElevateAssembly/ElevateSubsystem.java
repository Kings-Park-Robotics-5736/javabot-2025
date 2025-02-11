package frc.robot.subsystems.ElevateAssembly;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevateSubsystem extends SubsystemBase {
    private final ArmSubsystemFalcon m_arm;
    private final EndeffectorSubsystem m_endeffector;
    private final ElevatorSubsystemFalcon m_elevator;

    public ElevateSubsystem(){
        m_arm = new ArmSubsystemFalcon();
        m_endeffector = new EndeffectorSubsystem();
        m_elevator = new ElevatorSubsystemFalcon("Elevator");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Forward Limit Reached", m_endeffector.ForwardLimitReached());
        SmartDashboard.putBoolean("Reverse Limit Reached", m_endeffector.ReverseLimitReached());
        
    }

    //TODO: Make commands for the arm and elevator to go to each named position


    public Command sysIdArmQuasistatic(SysIdRoutine.Direction direction) {
        return m_arm.sysIdQuasistatic(direction);
    }

    public Command sysIdArmDynamic(SysIdRoutine.Direction direction) {
        return m_arm.sysIdDynamic(direction);
    }

    public Command sysIdElevatorQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevator.sysIdQuasistatic(direction);
    }

    public Command sysIdElevatorDynamic(SysIdRoutine.Direction direction) {
        return m_elevator.sysIdDynamic(direction);
    }

    public Command RunArmToPositionCommand(double position){
        return m_arm.RunArmToPositionCommand(position);
    }

    public Command RunArmUpManualSpeedCommand(DoubleSupplier speed){
        return m_arm.RunArmUpManualSpeedCommand(speed);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier speed){
        return m_arm.RunArmDownManualSpeedCommand(speed);
    }

    public Command RunElevatorManualSpeedCommand(DoubleSupplier speed){
        return m_elevator.RunElevatorManualSpeedCommand(speed);
    }

    public Command RunElevatorToPositionCommand(double position){
        return m_elevator.RunElevatorToPositionCommand(position);
    }

    public Command GoToIntake(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kOutofthewayPosition).andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle).andThen(m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake())));
    }

    public Command ScoreL4(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kL4Position).andThen(m_arm.RunArmToPositionCommand(ArmConstants.L4Angle).andThen(m_endeffector.Score()));
    }

    
    
}
