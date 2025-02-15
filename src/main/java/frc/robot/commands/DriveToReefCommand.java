package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.field.ScoringPositions.ScoreHeight;
import frc.robot.field.ScoringPositions.ScoreLocation;
import frc.robot.field.ScoringPositions.ScorePositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.vision.PiCamera;

public class DriveToReefCommand extends Command{
    private DriveSubsystem m_drive;
    private ScoreLocation m_scoreLocation;
    private ScoreHeight m_scoreHeight;
    private Command m_pathFollowCommand;
    private Pose2d m_targetPose;


    public DriveToReefCommand(DriveSubsystem robot_drive, ScoreLocation score_location, ScoreHeight score_height){
        m_drive = robot_drive;
        m_scoreLocation = score_location;
        m_scoreHeight = score_height;
    }   


    @Override
    public void initialize() {
        ScorePositions closestTarget = MathUtils.getClosestScoringTarget(m_drive.getPose());
        m_pathFollowCommand = TrajectoryCommandsFactory.getPathFollowCommandFromPositionLocAndHeight(m_drive, closestTarget,m_scoreLocation, m_scoreHeight);
        m_pathFollowCommand.initialize();
    }
    @Override
    public void execute() {
        m_pathFollowCommand.execute();
       
    }
    @Override
    public void end(boolean interrupted){
        m_pathFollowCommand.end(interrupted);

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if (m_pathFollowCommand.isFinished()){
            return true;
        }
        return false;
    }
}
