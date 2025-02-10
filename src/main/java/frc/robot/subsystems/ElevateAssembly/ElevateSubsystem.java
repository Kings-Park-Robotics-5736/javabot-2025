package frc.robot.subsystems.ElevateAssembly;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevateSubsystem extends SubsystemBase {
    private final ArmSubsystemFalcon m_arm;
    private final ElevatorSubsystem m_elevator;
    private final EndeffectorSubsystem m_endeffector;

    public ElevateSubsystem(){
        m_arm = new ArmSubsystemFalcon();
        m_elevator = new ElevatorSubsystem("elevator");
        m_endeffector = new EndeffectorSubsystem();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Forward Limit Reached", m_endeffector.ForwardLimitReached());
        SmartDashboard.putBoolean("Reverse Limit Reached", m_endeffector.ReverseLimitReached());
        
    }

    //TODO: Make commands for the arm and elevator to go to each named position




    
    
}
