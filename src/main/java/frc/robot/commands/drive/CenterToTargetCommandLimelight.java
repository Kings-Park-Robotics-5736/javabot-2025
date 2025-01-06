package frc.robot.commands.drive;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.Limelight;

/**
 * @brief This command centers the robot on the target USING THE LIMELIGHT.
 *        It does NOT drive to it. It is useful for a user to manually hold
 *        down a button, for instance, and manually drive to an object
 * 
 * @note Because we want the user to still be able to translate robot, we will
 *       not require robotDrive as a subsystem. Otherwise all joystick controls
 *       will be locked out. As such, we will manually lock out the rotate
 *       joystick.
 */
public class CenterToTargetCommandLimelight extends CenterToTargetCommand {

    private final double LIMELIGHT_ERROR_THRESH = 2;

    private Limelight m_limelight;
    private long m_lastUpdate;

    public CenterToTargetCommandLimelight(DriveSubsystem robot_drive,
            Limelight limelight, boolean infinite) {

        super(robot_drive, infinite); //call the parent constructor.
        this.m_limelight = limelight;
        this.m_lastUpdate = 0;

    }

    @Override
    public void initialize() {
        m_limelight.setReflectivePipeline();
        super.initialize();

    }

    @Override
    public void end(boolean interrupted) {
        m_limelight.setAprilTagPipeline();
        super.end(interrupted);
    }

    @Override
    public void execute() {
            
      
        long lastUpdate = m_limelight.getLastOffsetXChange();
        boolean useCameraMeasurement = false;
        if (lastUpdate != m_lastUpdate) {
            m_lastUpdate = lastUpdate;
            useCameraMeasurement = true;
        }
        centerOnTarget(m_limelight.getTargetOffsetX(), useCameraMeasurement);
        
        
    }

    protected boolean checkTurningDone() {
        return Math.abs(m_limelight.getTargetOffsetX()) < LIMELIGHT_ERROR_THRESH;
    }

}
