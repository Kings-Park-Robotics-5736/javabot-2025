package frc.robot.subsystems.ElevateAssembly;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevateSubsystem extends SubsystemBase {
    private final ArmSubsystemFalcon m_arm;
    private final ElevatorSubsystem m_elevator;

    public ElevateSubsystem(){
        m_arm = new ArmSubsystemFalcon();
        m_elevator = new ElevatorSubsystem("elevator");
    }

    @Override
    public void periodic() {
    }

    //TODO: Make commands for the arm and elevator to go to each named position
    
}
