package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.drive.PathPlanFromDynamicStartCommand;
import frc.robot.field.ScoringPositions;
import frc.robot.field.ScoringPositions.ScoreHeight;
import frc.robot.field.ScoringPositions.ScoreLocation;
import frc.robot.field.ScoringPositions.ScorePositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;

public class TrajectoryCommandsFactory {

    private static PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));


    /*****************************
     * Helpers
     *****************************/
    public static PathPlannerPath getPathFromFile(String pathName) {
        PathPlannerPath path = null;
        try{
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e){
            System.out.println("Error loading path " + pathName);
        }
        return path;
    }

    public static String getPathName(ScorePositions position, ScoreLocation location, ScoreHeight height) {
        String pathName = "";
        String alphaPosition = ScoringPositions.ScoreClockPositionToAlphaName(position);
        String alphaLocation = location == ScoreLocation.LEFT ? "Left" : "Right";
        String alphaHeight = (height == ScoreHeight.L1 || height == ScoreHeight.L2 || height == ScoreHeight.L3) ? "L23" : height == ScoreHeight.L4 ? "L4" : "" ;

        pathName = alphaPosition + alphaHeight + alphaLocation + "GEN";

        return pathName;
    }
    

    /*******************************
     * Commands
     *******************************/

    public static Command generatePPPathFindToPose(DriveSubsystem robotDrive, Pose2d endPos) {

        return  AutoBuilder.pathfindToPose(
            endPos,
            constraints,
            0.0 // Goal end velocity in meters/sec
        );
    }


    public static Command generatePPPathFindToPath(DriveSubsystem robotDrive, String pathName) {

        PathPlannerPath path = getPathFromFile(pathName);

        if( path != null){
            Pose2d endPose = new Pose2d(path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getX(), 
            path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getY(),
            path.getGoalEndState().rotation());

            return Commands.runOnce(()->System.out.println("Running Auto " + pathName + "with end pose " + endPose.toString())).andThen(AutoBuilder.pathfindThenFollowPath(path, constraints));
        }
        return Commands.run(() -> {
            System.out.println("Error loading path");
        });
    }

    public static Command generatePPPathFindToPathThenAlign(DriveSubsystem robotDrive, String pathName) {

        PathPlannerPath path = getPathFromFile(pathName);
      

        if( path != null){
            Pose2d endPose = new Pose2d(path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getX(), 
            path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getY(),
            path.getGoalEndState().rotation());

            return (Commands.runOnce(()->System.out.println("Running PP Path Find W/ Align to path " + pathName)) 
                    .andThen(AutoBuilder.pathfindThenFollowPath(path, constraints))
                    .andThen(new PathPlanFromDynamicStartCommand(
                        () -> robotDrive.getPose(),
                        robotDrive,
                        endPose,
                        new ArrayList<PathPoint>(),
                        true
                    ))
                    .andThen(Commands.runOnce(()->System.out.println("Done Running PP Path Find W/ Align to path " + pathName)))).withName("PP Path Find W/ Align to path " + pathName);
        }
        return Commands.run(() -> {
            System.out.println("Error loading path");
        });
    }

    /**
     * This method is used by PathPlanFromDynamicStartCommand to create a path, should not be used directly
     */
    public static Command generatePPPathToPose( List<Waypoint> waypoints, Rotation2d endRotation ) {

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
      
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, endRotation) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return AutoBuilder.followPath(path); 
    }



    public static Command getPathFollowCommandFromPositionLocAndHeight(DriveSubsystem robotDrive, ScorePositions position, ScoreLocation location, ScoreHeight height ) {
        return generatePPPathFindToPathThenAlign(robotDrive, getPathName(position, location, height));
    }


    public static Command getPathFollowCommandClearAlgae(DriveSubsystem robotDrive, ScorePositions position) {
        return generatePPPathFindToPathThenAlign(robotDrive, "CLEAR" + ScoringPositions.ScoreClockPositionToAlphaName(position) + "GEN");
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


    public static final Map<String, Command> getAllScoringCommands(DriveSubsystem robotDrive, Boolean left, Boolean top) {
        Map<String, Command> commands = new HashMap<>();
        for (ScorePositions pos : ScoringPositions.scorePositionsList) {
            for (ScoreLocation loc : ScoreLocation.values()) {
                for (ScoreHeight height : ScoreHeight.values()) {
                    commands.put(pos.toString() + loc.toString() + height.toString(), getPathFollowCommandFromPositionLocAndHeight(robotDrive, pos, loc, height));
                }
            }
        }

        return commands;
    }


    public static final Map<String, Command> getAllClearingCommands(DriveSubsystem robotDrive) {
        Map<String, Command> commands = new HashMap<>();
        for (ScorePositions pos : ScoringPositions.scorePositionsList) {
                commands.put("CLEAR" + pos.toString(), getPathFollowCommandClearAlgae(robotDrive,pos));
        }

        return commands;
    }


    public static final Command getClearingSelectedCommand(DriveSubsystem robotDrive, Supplier<String> selectedCommandSupplier){
        Map <String, Command> commands = getAllClearingCommands(robotDrive);
        return new SelectCommand<>(
            commands,
            selectedCommandSupplier
        );
    }

    public static final Command getScoringClosestCommand(DriveSubsystem robotDrive, Boolean left, Boolean top){
        return getScoringSelectedCommand(robotDrive, left, top, MathUtils.getClosestScoringTargetSupplier(robotDrive, left, top));
    }

    public static final Command getScoringSelectedCommand(DriveSubsystem robotDrive, Boolean left, Boolean top, Supplier<String> selectedCommandSupplier){
        Map <String, Command> commands = getAllScoringCommands(robotDrive, left, top);
        return new SelectCommand<>(
            commands,
            selectedCommandSupplier
        );
    }

    public static final Command GoToRightIntake(DriveSubsystem robotDrive){
        return generatePPPathFindToPathThenAlign(robotDrive, "IntakeRIGHT");
    }

    public static final Command GoToLeftIntake(DriveSubsystem robotDrive){
        return generatePPPathFindToPathThenAlign(robotDrive, "IntakeLEFT");
    }

    public static final Command GoToLeftCage(DriveSubsystem robotDrive){
        return Commands.runOnce(()->System.out.println("DRIVING TO LEFT CAGE")).andThen(generatePPPathFindToPathThenAlign(robotDrive, "LeftClimb"));
    }

    public static final Command GoToRightCage(DriveSubsystem robotDrive){
        return Commands.runOnce(()->System.out.println("DRIVING TO RIGHT CAGE")).andThen(generatePPPathFindToPathThenAlign(robotDrive, "RightClimb"));
    }

    public static final Command GoToMiddleCage(DriveSubsystem robotDrive){
        return Commands.runOnce(()->System.out.println("DRIVING TO MIDLE CAGE")).andThen(generatePPPathFindToPathThenAlign(robotDrive, "MiddleClimb"));
    }

    public static final Map<String, Command> getAllCageCommands(DriveSubsystem robotDrive) {
        Map<String, Command> commands = new HashMap<>();
        commands.put("CAGE1", GoToLeftCage(robotDrive));
        commands.put("CAGE2", GoToMiddleCage(robotDrive));
        commands.put("CAGE3", GoToRightCage(robotDrive));


        return commands;
    }

    public static final Command goToSelectedCageCommand(DriveSubsystem robotDrive, Supplier<String> selectedCommandSupplier){
        Map <String, Command> commands = getAllCageCommands(robotDrive);
        return new SelectCommand<>(
            commands,
            selectedCommandSupplier
        ).andThen(()->System.out.println("Selected" +  selectedCommandSupplier.get()));
    }

    


    


}
