package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.drive.CenterToGoalCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Types.GoalType;

public class TrajectoryCommandsFactory {

    /**
     * @brief calculate a trajectory given a start and an end
     * @param start
     * @param end
     * @return
     */

    public static Command generatePPTrajectoryCommand(DriveSubsystem robotDrive, Pose2d endPos) {

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

    public static Command getPathFollowCommandAmp() {
        // Load the path you want to follow using its name in the GUI
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile("Amp Path");   
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path); 
        } catch (Exception e){
            System.out.println("Error loading path");
        }

        return Commands.run(() -> {
            System.out.println("Error loading path");
        });

   
    }


    public static Command DriveToAmp(DriveSubsystem robotDrive){
        return getPathFollowCommandAmp().andThen(getPathFollowCommandAmp()).andThen(new CenterToGoalCommand(robotDrive, false, false, GoalType.AMP, Math.toRadians(4)));
    }

    public static Command getPathFollowCommandTrap() {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Trap Path");

            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        }
        catch (Exception e){
            System.out.println("Error loading path");
        }
        return Commands.run(() -> {
            System.out.println("Error loading path");
        });
    }


}
