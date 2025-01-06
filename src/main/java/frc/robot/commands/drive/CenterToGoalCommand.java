package frc.robot.commands.drive;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Types.GoalType;
import frc.robot.Constants.CenterToFieldPositionConstants;
import frc.robot.field.ScoringPositions;



/**
 * @brief This command centers the robot to a specific point on the field. It
 *        does NOT drive to it
 * 
 * @note Because we want the user to still be able to translate robot, we will
 *       not require robotDrive as a subsystem. Otherwise all joystick controls
 *       will be locked out. As such, we will manually lock out the rotate
 *       joystick.
 */
public class  CenterToGoalCommand extends CenterToTargetCommand {

    private static final double POSE_ERROR_THRESH = Math.toRadians(4);
    private static double m_goal_rotation = 0;
    private boolean m_oppositeGoal;
    private GoalType m_goalType;
    private int m_stale_counter;
    private double m_threshold;
    private double stale_value;


    public CenterToGoalCommand(DriveSubsystem robot_drive, boolean infinite, boolean OppositeGoal, GoalType goal, double threshold) {
       
        // call the parent constructor.
        super(robot_drive, infinite, new TrapezoidProfile.Constraints(
            CenterToFieldPositionConstants.kMaxSpeedMetersPerSecond, CenterToFieldPositionConstants.kMaxAccelerationMetersPerSecondSquared), CenterToFieldPositionConstants.kPidValues); 
        m_oppositeGoal = OppositeGoal;
        m_goalType = goal;
        m_threshold = threshold;
       
    }

    public CenterToGoalCommand(DriveSubsystem robot_drive, boolean infinite) {
        this(robot_drive, infinite, false, GoalType.SPEAKER,POSE_ERROR_THRESH);
    }

    public CenterToGoalCommand(DriveSubsystem robot_drive, boolean infinite,boolean OppositeGoal) {
        this(robot_drive, infinite, OppositeGoal, GoalType.SPEAKER,POSE_ERROR_THRESH);
    }

    @Override
    public void initialize() {
        m_drive.setJoystickRotateLockout(true, true);
        m_controller_theta.reset(m_drive.getPose().getRotation().getRadians());
        m_controller_theta.setTolerance(0.06);
        m_controller_theta.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("----------------Centering to target inverse = " + m_oppositeGoal + " -----------------------");
        calculateRotation();
        System.out.println("----------------Desired Goal is  " + m_goal_rotation + " -----------------------");
        m_stale_counter = 0;
        stale_value = 0;

    }


    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
        m_drive.setJoystickRotateLockout(false);
        m_drive.setRotateLockoutValue(0);
        stop();
    }

    @Override
    protected void centerOnTarget(double angle, boolean useCameraMeasurement) {
        double rotationVel = DriveCommandsCommon.calculateRotationToFieldPos(
                m_drive.getPose().getRotation().getRadians(),
                useCameraMeasurement,
                angle, m_controller_theta);
        m_drive.setRotateLockoutValue(rotationVel);
        m_drive.drive(0, 0, rotationVel, true, false);
    }

    public double calculateRotation(){
        Pose2d robotPose = m_drive.getPose();
        Pose2d scoringPos;
        double rotationOffset = 0;

        var alliance = DriverStation.getAlliance();
        if(m_goalType == GoalType.SPEAKER){
            if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Blue || (m_oppositeGoal && alliance.get() ==DriverStation.Alliance.Red))) {
                scoringPos = ScoringPositions.kBlueScoringPosition;
                rotationOffset += Units.degreesToRadians(180); //when facing the blue side of the field, that is 180 deg.
            } else if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red|| (m_oppositeGoal && alliance.get() ==DriverStation.Alliance.Blue))) {
                scoringPos = ScoringPositions.kRedScoringPosition;
            } else {
                return 0;
            }

            var xDelta = robotPose.getTranslation().getX() - scoringPos.getTranslation().getX();
            var yDelta = robotPose.getTranslation().getY() - scoringPos.getTranslation().getY();
            var angleToTarget = Math.atan(yDelta / xDelta) + rotationOffset; // normally tan is x/y, but in frc coords, it
                                                                            // is y / x
            m_goal_rotation = angleToTarget;
        }else{
            if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Blue || (m_oppositeGoal && alliance.get() ==DriverStation.Alliance.Red))) {
                m_goal_rotation = Math.toRadians(90);
            } else if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red|| (m_oppositeGoal && alliance.get() ==DriverStation.Alliance.Blue))) {
                m_goal_rotation = Math.toRadians(-90);
            } else {
                return 0;
            }
        }
        
        return m_goal_rotation;
    }

    @Override
    public void execute() {

        // calculate the angle of our current pose to the target
        var angleToTarget = calculateRotation();
        System.out.println("Angle to target is " + angleToTarget);
        centerOnTarget(angleToTarget, true);

    }

    protected boolean checkTurningDone() {
        
        if(Math.abs(stale_value - m_drive.getPose().getRotation().getRadians() ) < Math.toRadians(.25)){
            m_stale_counter++;
        }else{
            stale_value = m_drive.getPose().getRotation().getRadians();
            m_stale_counter = 0;
        }
        System.out.println("Stale Counter = " + m_stale_counter + ", angle offset = " + Math.abs(m_goal_rotation - m_drive.getPose().getRotation().getRadians() ));
        return Math.abs(m_goal_rotation - m_drive.getPose().getRotation().getRadians() ) < m_threshold || m_stale_counter > 20;
    }

    public static boolean checkTurningDoneStatic(DriveSubsystem drive){
        return Math.abs(m_goal_rotation - drive.getPose().getRotation().getRadians() ) < POSE_ERROR_THRESH;
    }

}