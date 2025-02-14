package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.CenterToGoalCommand;
import frc.robot.field.ScoringPositions.ScoreLocation;
import frc.robot.field.ScoringPositions;
import frc.robot.field.ScoringPositions.ScorePositions;
import frc.robot.field.ScoringPositions.ScoreHeight;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Types.GoalType;
import frc.robot.utils.MathUtils;

public class TrajectoryCommandsFactory {

    /**
     * @brief calculate a trajectory given a start and an end
     * @param start
     * @param end
     * @return
     */

    public static Command generatePPPathFindToPose(DriveSubsystem robotDrive, Pose2d endPos) {

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return  AutoBuilder.pathfindToPose(
            endPos,
            constraints,
            0.0 // Goal end velocity in meters/sec
        );
    }


    public static PathPlannerPath getPathFromFile(String pathName) {
        PathPlannerPath path = null;
        try{
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e){
            System.out.println("Error loading path");
        }
        return path;
    }

    public static Command generatePPPathFindToPath(DriveSubsystem robotDrive, String pathName) {

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        PathPlannerPath path = getPathFromFile(pathName);

        if( path != null){
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        }
        return Commands.run(() -> {
            System.out.println("Error loading path");
        });
    }

    public static Command generatePPPathToPose(DriveSubsystem robotDrive, Pose2d endPos) {

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(endPos.getX(), endPos.getY(), endPos.getRotation()) // The start pose of the path
        );
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return AutoBuilder.followPath(path); 
    }

    public static String getPathName(ScorePositions position, ScoreLocation location, ScoreHeight height) {
        String pathName = "";
        String alphaPosition = ScoringPositions.ScoreClockPositionToAlphaName(position);
        String alphaLocation = location == ScoreLocation.LEFT ? "Left" : "Right";
        String alphaHeight = (height == ScoreHeight.L1 || height == ScoreHeight.L2) ? "L23" : height == ScoreHeight.L4 ? "L4" : "" ;

        pathName = alphaPosition + alphaHeight + alphaLocation + "GEN.path";

        return pathName;
    }

    public static Command getPathFollowCommand(DriveSubsystem robotDrive,ScorePositions position, ScoreLocation location, ScoreHeight height ) {
        return generatePPPathFindToPath(robotDrive, getPathName(position, location, height));
    }

    public static Pose2d getPathFinishingPose(ScorePositions position, ScoreLocation location, ScoreHeight height){

        PathPlannerPath path = getPathFromFile(getPathName(position, location, height));
        if (path == null) {
            System.out.println("Error loading path");
            return new Pose2d();
        }
       return  new Pose2d(path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getX(), 
                                path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getY(),
                                path.getGoalEndState().rotation());

      
    }

    public static final Command getAllScoringCommands(DriveSubsystem robotDrive, BooleanSupplier left, BooleanSupplier top){
        Map <String, Command> commands = Collections.emptyMap();
        for (ScorePositions pos : ScoringPositions.scorePositionsList){
            for (ScoreLocation loc : ScoreLocation.values()){
                for (ScoreHeight height : ScoreHeight.values()){
                    commands.put(pos.toString() + loc.toString() + height.toString(), getPathFollowCommand(robotDrive, pos, loc, height));
                }
            }
        }
        
        return new SelectCommand<>(
            commands,
            MathUtils.getClosestScoringTargetSupplier(robotDrive, left, top)
        );
    }
    


    


}
