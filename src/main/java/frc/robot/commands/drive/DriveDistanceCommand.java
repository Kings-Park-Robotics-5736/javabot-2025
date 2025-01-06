package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CustomDriveDistanceCommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * @brief This command is used to drive a specific distance
 *        It will drive straight from wherever the robot is currently facing.
 */
public class DriveDistanceCommand extends Command {

    private final double m_meters;
    private final DriveSubsystem m_robotDrive;

    // Control the motion profile for the auto-commands for driving. This is kind-of
    // like a path following
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    private final ProfiledPIDController m_controller_x = new ProfiledPIDController(
            CustomDriveDistanceCommandConstants.kPidValues.p, CustomDriveDistanceCommandConstants.kPidValues.i,
            CustomDriveDistanceCommandConstants.kPidValues.d, m_constraints,
            Constants.kDt);
    private final ProfiledPIDController m_controller_y = new ProfiledPIDController(
            CustomDriveDistanceCommandConstants.kPidValues.p, CustomDriveDistanceCommandConstants.kPidValues.i,
            CustomDriveDistanceCommandConstants.kPidValues.d, m_constraints,
            Constants.kDt);

    public DriveDistanceCommand(DriveSubsystem robotDrive, double meters) {
        m_meters = meters;
        m_robotDrive = robotDrive;
        addRequirements(robotDrive); // this effectively locks out the joystick

    }

    @Override
    public void initialize() {
        InitMotionProfile(setpointToX(m_meters), setpointToY(m_meters));
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.drive(0, 0, 0, true);
        m_robotDrive.setJoystickTranslateLockout(false);
    }

    @Override
    public void execute() {
        double pid_valX = m_controller_x.calculate(m_robotDrive.getPose().getX());
        double vel_pid_valX = pid_valX * DriveConstants.kMaxSpeedMetersPerSecond;

        double pid_valY = m_controller_y.calculate(m_robotDrive.getPose().getY());
        double vel_pid_valY = pid_valY * DriveConstants.kMaxSpeedMetersPerSecond;

        double finalVelX = m_controller_x.getSetpoint().velocity + vel_pid_valX;
        double finalVelY = m_controller_y.getSetpoint().velocity + vel_pid_valY;

        m_robotDrive.drive(finalVelX, finalVelY, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return m_controller_x.atGoal() && m_controller_y.atGoal();
    }

    /**
     * @brief Determines the x value of the setpoint based on the current heading
     * @param setpoint in meters
     * @return
     */
    private double setpointToX(double setpoint) {
        return setpoint * m_robotDrive.getPose().getRotation().getCos() + m_robotDrive.getPose().getX();
    }

    /**
     * @brief Determines the y value of the setpoint based on the current heading
     * @param setpoint in meters
     * @return
     */
    private double setpointToY(double setpoint) {
        return setpoint * m_robotDrive.getPose().getRotation().getSin() + m_robotDrive.getPose().getY();
    }

    /**
     * @brief Initialize the motion profile for the PID controller
     * @param setpointX
     * @param setpointY
     */
    private void InitMotionProfile(double setpointX, double setpointY) {

        m_controller_x.reset(m_robotDrive.getPose().getX());
        m_controller_x.setTolerance(.05);
        m_controller_x.setGoal(new TrapezoidProfile.State(setpointX, 0));

        m_controller_y.reset(m_robotDrive.getPose().getY());
        m_controller_y.setTolerance(.05);
        m_controller_y.setGoal(new TrapezoidProfile.State(setpointY, 0));

        m_robotDrive.setJoystickTranslateLockout(true);// begin locking out the 0 values from the joystick

    }

}
