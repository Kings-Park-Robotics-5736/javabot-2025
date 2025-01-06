package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveToTargetCommandConstants;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.PiCamera;

/**
 * @brief This command is like CenterToTarget, except it will also drive at a
 *        static speed until it reaches the target
 * 
 * @not Reaching target is defined as no longer being able to see it.
 */
public class DriveToTargetCommand extends Command {
    private DriveSubsystem m_drive;
    private boolean m_gotTarget;
    private int m_gotTargetCounter;
    private Pose2d m_startingPosition;
    private double m_startingV;

    private final PiCamera m_picam;
    private  double m_speed;
    private  DoubleSupplier m_speed_supplier;
    private final boolean m_dynamic_speed;
    private final double m_maxDistance;
    private TrapezoidProfile speed_profile;
    private int m_iterationCounter;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);

    private final ProfiledPIDController m_controller_theta = new ProfiledPIDController(
            DriveToTargetCommandConstants.kPidValues.p, DriveToTargetCommandConstants.kPidValues.i,
            DriveToTargetCommandConstants.kPidValues.d, m_constraints,
            Constants.kDt);

    public DriveToTargetCommand(DriveSubsystem robot_drive, PiCamera picam, double speed, double maxDistance) {

        this.m_drive = robot_drive;
        this.m_picam = picam;
        this.m_speed = speed;
        this.m_maxDistance = maxDistance;
        this.m_dynamic_speed = false;


        addRequirements(m_drive);

    }

    public DriveToTargetCommand(DriveSubsystem robot_drive, PiCamera picam, DoubleSupplier speed, double maxDistance) {

        this.m_drive = robot_drive;
        this.m_picam = picam;
        this.m_speed_supplier = speed;
        this.m_maxDistance = maxDistance;
        this.m_dynamic_speed = true;

        addRequirements(m_drive);

    }

    @Override
    public void initialize() {

        //Lock out drive controls
        m_drive.setJoystickRotateLockout(true);
        m_drive.setJoystickTranslateLockout(true);

        m_gotTarget = false;
        m_gotTargetCounter = 0;
        m_iterationCounter = 0;

        //acuire current pose to check total distance travelled
        m_startingPosition = m_drive.getPose();
        System.out.println("Start " + m_startingPosition.getX() + " ,  " + m_startingPosition.getY() + ", " + m_startingPosition.getRotation().getRadians());
        
        //configure motion controller
        m_controller_theta.reset(m_drive.getHeadingInRadians());
        m_controller_theta.setTolerance(0.01);
        var startingSpeeds = m_drive.getRobotRelativeSpeeds();
        m_startingV = Math.sqrt(startingSpeeds.vxMetersPerSecond *  startingSpeeds.vxMetersPerSecond + 
        startingSpeeds.vyMetersPerSecond *  startingSpeeds.vyMetersPerSecond);
    }

    @Override
    public void end(boolean interrupted) {

        //stop the robot from moving completely
        m_drive.drive(0, 0, 0, true);

        //unlock joysticks
        m_drive.setJoystickRotateLockout(false);
        m_drive.setJoystickTranslateLockout(false);
        m_drive.setRotateLockoutValue(0);
    }

    @Override
    public void execute() {

        if(!this.m_dynamic_speed){
            if(m_startingV < .20){
                m_startingV = .20;
            }
        
            m_iterationCounter++;
            if(m_iterationCounter %2 ==0 && m_startingV < m_speed){
                m_startingV += .075;
            }
            driveToTarget(m_startingV, m_maxDistance);
        }else{
            System.out.println(m_speed_supplier.getAsDouble());
            driveToTarget(m_speed_supplier.getAsDouble(), m_maxDistance);
        }
    }

    @Override
    public boolean isFinished() {
        return m_gotTarget;
    }

    /**
     * @brief  Calculate the total displacement from the starting position
     * @note   Uses euclidean distance formula based on x and y coordinates
     * @return total displacement
     */
    private double getTotalDisplacement() {
        var pose = m_drive.getPose(); // get current pose

        // euclidean distance formula (sqrt((x2-x1)^2 + (y2-y1)^2)
        return Math.sqrt((pose.getX() - m_startingPosition.getX()) * (pose.getX() - m_startingPosition.getX()) +
                (pose.getY() - m_startingPosition.getY()) * (pose.getY() - m_startingPosition.getY()));
    }


    public static double AngleDifference( double angle1, double angle2 )
    {
        double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }
    /**
     * @brief  Calculate the total rotation from the starting position
     * @return total rotation
     */
    private double getTotalRotation() {
        return Math.abs(AngleDifference(m_drive.getPose().getRotation().getRadians() ,m_startingPosition.getRotation().getRadians()));
    }

    /**
     * @brief  Drive to target using the PiCamera to center us
     * @param  speed: speed to drive at
     * @param  maxDistance: maximum distance to drive before stopping if we 'run away' to a target too far
     */
    private void driveToTarget(double speed, double maxDistance) {
        var piAngle = m_picam.getPiCamAngle(); // get angle from PiCamera interface
        if (Math.abs(piAngle) < 50) { //only trust angles that are in range -50deg to 50deg
            double rotationVel = DriveCommandsCommon.calculateRotationToTarget(m_drive.getHeadingInRadians(), true,
                    piAngle,
                    m_controller_theta);

            if (rotationVel > -100) {

                m_drive.setRotateLockoutValue(rotationVel);
                m_drive.drive(speed, 0, rotationVel, false, false);
                m_gotTargetCounter = 0;
            } else {
                m_gotTargetCounter++;

            }
        } else {
            m_gotTargetCounter++;
        }

        // if we can no longer see the target for a while, we are done
        if (m_gotTargetCounter > DriveToTargetCommandConstants.kGotTargetThreshold) {
            m_gotTarget = true;
            System.out.println("---------------Lost Target-----------------");
        }
        
        if( m_iterationCounter %5 == 0){
            System.out.println(m_drive.getPose().getX() + " ,  " + m_drive.getPose().getY() + ", " +m_drive.getPose().getRotation().getRadians());
        }
        if (maxDistance > 0 && getTotalDisplacement() > maxDistance)
                /*|| getTotalRotation() > DriveToTargetCommandConstants.kMaxRotation)*/ {
                    System.out.println(m_drive.getPose().getX() + " ,  " + m_drive.getPose().getY() + ", " +m_drive.getPose().getRotation().getRadians());
            // this is an error check to ensure we can't just run-away if we are fully auto
            System.out.println("---------------Overran-----------------");
            m_gotTarget = true;
        }

    }
}