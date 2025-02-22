package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TrajectoryCommandsFactory;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * @brief this is a wrapper around the path planning lib to allow us to go to a
 *        target based on our current starting position
 * 
 * @note Usual pplib requires giving the trajectory on command creation ('new'
 *       operator). This wrapper allows us to delay that until we call
 *       initialize().
 */
public class PathPlanFromDynamicStartCommand extends Command {

    private Command m_pathFollowCommand;
    private final Supplier<Pose2d> m_initialPoseSupplier;
    private final DriveSubsystem m_robotDrive;
    private final boolean m_continueTillAtPos;

    private Pose2d m_endPose;
    private ArrayList<PathPoint> m_mid_poses;
    private int m_pathPlannerDoneCounter;
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    /**
     * @param initialPoseSupplier method to obtain the current robot pose to
     *                            determine initial state.
     * @param robotDrive
     * @param endPose             the final pose to drive to
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose) {
        this(initialPoseSupplier, robotDrive, endPose, new ArrayList<PathPoint>(), false);

    }

    /**
     * @brief This constructor allows you to specify a list of intermediate points
     * @param initialPoseSupplier method to obtain the current robot pose to
     * @param robotDrive
     * @param endPose             the final pose to drive to
     * @param midPoses            a list of intermediate points to drive to along
     *                            the way
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose, ArrayList<PathPoint> midPoses) {
        this(initialPoseSupplier, robotDrive, endPose, midPoses, false);

    }

    /**
     * @param initialPoseSupplier
     * @param robotDrive
     * @param endPose
     * @param continueTillAtPos   whether to continue till we are at the final
     *                            position, or stop when pathplanner says time is up
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose,boolean continueTillAtPos) {
        this(initialPoseSupplier, robotDrive, endPose, new ArrayList<PathPoint>(), continueTillAtPos);

    }

    /**
     * @brief fully loaded constructor
     * @param initialPoseSupplier
     * @param robotDrive
     * @param endPose
     * @param midPoses
     * @param usePid
     * @param continueTillAtPos
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose, ArrayList<PathPoint> midPoses,  boolean continueTillAtPos) {
        m_initialPoseSupplier = initialPoseSupplier;
        m_robotDrive = robotDrive;
        m_endPose = endPose;
        m_continueTillAtPos = continueTillAtPos;
        m_mid_poses = midPoses;
        addRequirements(robotDrive); // this effectively locks out the joystick controlls.
        // without this, the 0,0 from the joystick still happens, causing jittery
        // movement
    }

    /**
     * @brief allows you to set the end pose after the command has been created
     * @note Must be set before initialize() is called (command is started) for it to take effect
     * @param endPose
     */
    public void SetEndPose(Pose2d endPose) {
        m_endPose = endPose;
    }

    @Override
    public void initialize() {

        var alliance = DriverStation.getAlliance();
        //guard against double flipping - if pos < 8.5, we are on blue, otherwise already on red
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && m_endPose.getX() < 8.5) {
            m_endPose = FlippingUtil.flipFieldPose(m_endPose);
        }
         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            m_initialPoseSupplier.get(),
            new Pose2d(m_endPose.getX(), m_endPose.getY(), m_endPose.getRotation()) // The start pose of the path
        );
        System.out.println("PathPlanDynamicStart Init Driving to " + m_endPose.getX() + ", " + m_endPose.getY() );

        //calculate distance between start and end points
        double distance = Math.sqrt(Math.pow(m_endPose.getX() - m_initialPoseSupplier.get().getX(), 2) + Math.pow(m_endPose.getY() - m_initialPoseSupplier.get().getY(), 2));

        if(distance > 0.0254){
            // delay trajectory creation until this initialize.
            m_pathFollowCommand = TrajectoryCommandsFactory.generatePPPathToPose(waypoints, m_endPose.getRotation());
            try{
                m_pathFollowCommand.initialize();
            }catch(Exception e){
                m_pathFollowCommand = null;
            }
        } else {
            m_pathFollowCommand = null;
        }
        m_pathPlannerDoneCounter = 0;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PathPlanDynamicStart END. Goal pose: " + m_endPose.toString() + ", Actual pose: " + m_robotDrive.getPose().toString());
        if(m_pathFollowCommand != null){
            m_pathFollowCommand.end(interrupted);
        }
    }

    @Override
    public void execute() {
        if(m_pathFollowCommand != null){
            m_pathFollowCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {

        if(m_pathFollowCommand == null){
            return true;
        }

        if (m_continueTillAtPos) {
            if (m_pathFollowCommand.isFinished()) {
                m_pathPlannerDoneCounter++;
            }
            return m_pathPlannerDoneCounter > 25 || (Math.abs(m_robotDrive.getPose().getX() - m_endPose.getX()) < .02
                    && Math.abs(m_robotDrive.getPose().getY() - m_endPose.getY()) < .02);
        } else {
            return m_pathFollowCommand.isFinished();
        }
    }

}