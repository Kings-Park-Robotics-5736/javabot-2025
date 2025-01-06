package frc.robot.subsystems.launcherAssembly.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SparkMaxUtils;
public class ArmSubsystemNEO extends SubsystemBase {

    private final SparkMax m_follower;
    private final SparkMax m_leader;

    private SparkMaxConfig m_leaderMotorConfig;
    private SparkMaxConfig m_followerMotorConfig;


    private boolean emergencyStop;

    private int staleCounter = 0;
    private double lastPosition = 0;
    private boolean manualControl = true;
    private double startTime = 0;
    private double armManualOffset = 0;

    private RelativeEncoder m_encoder;
    private double encoderOffset;



    private ArmFeedforward m_feedforward;
    private TrapezoidProfile profile;

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_distance = Radians.mutable(0);
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
    private final SysIdRoutine m_sysIdRoutine;
    private double m_globalSetpoint;

    private SparkClosedLoopController m_closedLoopController;

    private double usedP;
    private double usedI;
    private double usedD;
    private double usedV;
    private double usedG;
    private TrapezoidProfile.State m_prior_iteration_setpoint;

    private DigitalInput m_noteSensor;
    private RelativeEncoder m_alternateEncoder;
    private AbsoluteEncoder m_AbsoluteEncoder;

    private static final int kCPR = 8192;

    public ArmSubsystemNEO() {

        m_leader = new SparkMax(ArmConstants.kLeaderDeviceId, MotorType.kBrushless);
        m_follower = new SparkMax(ArmConstants.kFollowerDeviceId, MotorType.kBrushless);
        m_leaderMotorConfig = new SparkMaxConfig();
        m_followerMotorConfig = new SparkMaxConfig();
        m_noteSensor = new DigitalInput(ArmConstants.kNoteSensorDIO);

        
        m_feedforward = new ArmFeedforward(ArmConstants.kFFValues.ks, ArmConstants.kFFValues.kg,
                ArmConstants.kFFValues.kv, ArmConstants.kFFValues.ka);

       
        m_leaderMotorConfig.idleMode(IdleMode.kBrake);
       

        m_leaderMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(ArmConstants.kPidValues.p)
        .i(ArmConstants.kPidValues.i)
        .d(ArmConstants.kPidValues.d)
        .iZone(Math.toRadians(3))
        .velocityFF(0)
        .outputRange(-.5, .5);

        m_leaderMotorConfig.alternateEncoder
        .countsPerRevolution(kCPR)
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60)
        .inverted(false);

        m_followerMotorConfig.idleMode(IdleMode.kBrake);
        m_followerMotorConfig.follow(m_leader);


        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);
        SparkMaxUtils.ApplySparkMaxConfig(m_follower, m_followerMotorConfig);

        m_alternateEncoder = m_leader.getAlternateEncoder();

        encoderOffset = Math.toRadians(ArmConstants.armEncoderOffsetAngleDegrees);

        m_encoder = m_leader.getEncoder();

        m_closedLoopController = m_leader.getClosedLoopController();

     


        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(null, Volts.of(2), null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            // CANNOT use set voltage, it does not work. This normalizes the voltage between
                            // -1 and 0 and 1
                            m_leader.set(-volts.in(Volts) / RobotController.getBatteryVoltage());
                        },
                        log -> {
                            log.motor(("Arm"))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(
                                        getArmAngleRadians(),
                                            Radians))
                                    .angularVelocity(
                                            m_velocity.mut_replace(
                                                    getArmAngleVelocityRPS(),
                                                    RadiansPerSecond));

                        },
                        this));

        InitMotionProfile(getArmPosition());


        SmartDashboard.putNumber("ARM P", ArmConstants.kPidValues.p);
        SmartDashboard.putNumber("ARM I", ArmConstants.kPidValues.i);
        SmartDashboard.putNumber("ARM D", ArmConstants.kPidValues.d);
        SmartDashboard.putNumber("ARM V", ArmConstants.kFFValues.kv);
        SmartDashboard.putNumber("ARM G", ArmConstants.kFFValues.kg);

        usedP = ArmConstants.kPidValues.p;
        usedI = ArmConstants.kPidValues.i;
        usedD = ArmConstants.kPidValues.d;
        usedV = ArmConstants.kFFValues.kv;
        usedG = ArmConstants.kFFValues.kg;
        armManualOffset = 0;

    }

    public void resetAfterDisable(){
        System.out.println("Reset After Disable!!!");
        manualControl = true;
        setSpeed(0);
    }

    @Override
    public void periodic() {
       
        SmartDashboard.putNumber("Arm Angle Deg", Math.toDegrees(getArmAngleRadians()));
        SmartDashboard.putBoolean("Arm At Position",  armReachedTarget());

        var newP = SmartDashboard.getNumber("ARM P", usedP);
        var newI = SmartDashboard.getNumber("ARM I", usedI);
        var newD = SmartDashboard.getNumber("ARM D", usedD);
        var newV = SmartDashboard.getNumber("ARM V", usedV);
        var newG = SmartDashboard.getNumber("ARM G", usedG);
        
        if (newP != usedP) {
            usedP = newP;
            m_leaderMotorConfig.closedLoop.p(usedP);
            m_leader.configure(m_leaderMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("Using P of " + usedP);
        }

        if (newI != usedI) {
            usedI = newI;
            m_leaderMotorConfig.closedLoop.i(usedI);
            m_leader.configure(m_leaderMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("Using I of " + usedI);

        }

        if (newD != usedD) {
            usedD = newD;
            m_leaderMotorConfig.closedLoop.d(usedD);
            m_leader.configure(m_leaderMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("Using D of " + usedD);

        }

        if (newV != usedV && newV != 0) {
            usedV = newV;
            System.out.println("Using V of " + usedV);
            m_feedforward = new ArmFeedforward(ArmConstants.kFFValues.ks, usedG, usedV, ArmConstants.kFFValues.ka);
        }

        if (newG != usedG && newG != 0) {
            usedG = newG;
            System.out.println("Using G of " + usedG);
            m_feedforward = new ArmFeedforward(ArmConstants.kFFValues.ks, usedG, usedV, ArmConstants.kFFValues.ka);
        }

        if(!manualControl){
            RunArmToPos();
        }
    }

     public void RunArmToPos() {

        m_prior_iteration_setpoint = profile.calculate(0.02, m_prior_iteration_setpoint,new TrapezoidProfile.State(m_globalSetpoint, 0));//, m_setpoint,new TrapezoidProfile.State(m_globalSetpoint, 0));
        double ff =  m_feedforward.calculate(m_prior_iteration_setpoint.position, m_prior_iteration_setpoint.velocity);

        m_closedLoopController.setReference(m_prior_iteration_setpoint.position - encoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff,
                ArbFFUnits.kVoltage);


        SmartDashboard.putNumber("Arm FF Output", ff);
        SmartDashboard.putNumber("Arm Position Eror", Math.toDegrees(m_prior_iteration_setpoint.position - getArmAngleRadians()));
        SmartDashboard.putNumber("Arm Offset Manual", Math.toDegrees(armManualOffset));

        SmartDashboard.putNumber("Arm SetpointVelocity",m_prior_iteration_setpoint.velocity);
        SmartDashboard.putNumber("Arm SetpointPosition", Math.toDegrees(m_prior_iteration_setpoint.position));
        SmartDashboard.putNumber("Arm Global Setpoint ", Math.toDegrees(m_globalSetpoint));
        //System.out.println("Current Arm Angle: " + Math.toDegrees(getArmPosition()));
        
        //if we are about to overextend, instantly stop and go back to intake position.
        if (getArmAngleRadians() < ArmConstants.kLimits.high) {
            StopArm();
            InitMotionProfile(ArmConstants.intakeAngle);
        }
    }

    public void resetArmHomePosition(){
        encoderOffset = Math.toRadians(ArmConstants.armEncoderOffsetAngleDegrees) - (m_alternateEncoder.getPosition());
    }


    public boolean hasNote(){
        return !m_noteSensor.get();
    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_leader.set(speed);
    }

    /**
     * Stop the motor
     */
    public void StopArm() {
        System.out.println("Arm Stopping");
        setSpeed(0);
    }

    /*****************************
     * Getters for arm position
     *****************************/
    public double getArmPosition() {
        return getArmAngleRadians();
    }

    public double getArmAngleRadians() {
        return m_alternateEncoder.getPosition()
                + encoderOffset;
    }

    public double getArmAngleVelocityRPS(){
        return m_alternateEncoder.getVelocity();
    }

    public double getFalconAngleRadians() {
        return (m_encoder.getPosition() / 31.8181818) * 2 * Math.PI + Math.toRadians(33);
    }

    public double getArmAngleRadiansRaw() {
        return m_alternateEncoder.getPosition();
    }

    public double getFalconAngularVelocityRadiansPerSec() {
        return ((m_encoder.getVelocity() /60) / 31.8181818) * 2 * Math.PI;
    }



    private void InitMotionProfile(double setpoint, double maxAcceleration) {
      
        /*
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, maxAcceleration),
        new TrapezoidProfile.State(setpoint, 0),
        new TrapezoidProfile.State(getArmPosition(), 0));

        */

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, maxAcceleration));

        startTime = Timer.getFPGATimestamp();

        m_globalSetpoint = setpoint;
        m_prior_iteration_setpoint = new TrapezoidProfile.State(getArmPosition(), 0);
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint) {
        InitMotionProfile(setpoint,ArmConstants.kMaxAcceleration );
       
    }


    

    /**
     * @brief Checks if the arm has reached its target
     * @return true if the arm has reached its target, false otherwise
     */
    public Boolean armReachedTarget() {

        // check if the arm has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(getArmPosition() - lastPosition) < ArmConstants.kDiffThreshold) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = getArmPosition();

        // calculate the difference between the current position and the motion profile
        // final position
        double delta = Math.abs(getArmPosition() - m_globalSetpoint);

        // we say that the elevator has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < ArmConstants.kDiffThreshold
                || (delta < ArmConstants.kStaleTolerance && staleCounter > ArmConstants.kStaleThreshold);
    }

    public boolean armIsDown(){
        return  Math.abs(getArmPosition() - ArmConstants.intakeAngle) < Math.toRadians(1.5);
    }

    private Boolean isFinished() {
        System.out.println("emergency stop " + emergencyStop + ", " + "arm target: " + armReachedTarget());

        var isFinished = emergencyStop || armReachedTarget();
        return isFinished;
    }

    private Boolean isWithinLimits(double direction) {
        return getArmAngleRadians() > ArmConstants.kLimits.high || ( direction > 0);
    }


    private double sanitizePositionSetpoint(double setpoint){
        if (setpoint > ArmConstants.kLimits.low){
            setpoint = ArmConstants.kLimits.low;
        }
        if(setpoint < ArmConstants.kLimits.high){
            setpoint = ArmConstants.kLimits.high;
        }
        return setpoint;
    }


    public void RunArmToPosition(double setpoint){
        double sanitizedSetpoint = sanitizePositionSetpoint(setpoint);
        System.out.println("-----------------Starting Arm to position NO COMMMAND" + sanitizedSetpoint + " --------------");

        double maxAcceleration = ArmConstants.kMaxAcceleration;
        if(setpoint == ArmConstants.scorpionAngle){
            maxAcceleration = (double) 1.25*  Math.PI;
        }
        InitMotionProfile(sanitizedSetpoint,maxAcceleration);
        manualControl = false;
    }
    /**
     * 
     * @param setpoint the desired arm position IN RADIANS
     * @note When FinishWhenAtTargetSpeed is true, the StopShooter() should not be
     *       called when the command finishes.
     * @return
     */

    public Command RunArmToPositionCommand(double setpoint) {
        return new FunctionalCommand(
                () -> {
                    double sanitizedSetpoint = sanitizePositionSetpoint(setpoint);
                    System.out.println("-----------------Starting Arm to position " + sanitizedSetpoint + " --------------");

                    double maxAcceleration = ArmConstants.kMaxAcceleration;
                    if(setpoint == ArmConstants.scorpionAngle){
                        maxAcceleration = (double) 1.25*  Math.PI;
                    }
                    InitMotionProfile(sanitizedSetpoint,maxAcceleration);
                    manualControl = false;
                },
                () -> {},
                (interrupted) -> {
                    emergencyStop = false;
                },
                () -> {
                    return isFinished();
                }, this);
    }


    public void CalculateNewAutoAngle(DriveSubsystem robotDrive){
        double new_setpoint = MathUtils.angleRadiansToScoringTarget(robotDrive.getPose());
        double sanitizedSetpoint = sanitizePositionSetpoint(new_setpoint);
        if(sanitizedSetpoint != m_globalSetpoint){
            System.out.println("-----------------New Arm to Setpoint " + Math.toDegrees(sanitizedSetpoint) + " --------------");
            
        }
        m_globalSetpoint = sanitizedSetpoint + armManualOffset;
    }

    public void UpdateAngleManually(double diff){
        m_globalSetpoint += diff;
        InitMotionProfile(m_globalSetpoint);
        armManualOffset +=diff;
    }


    public Command RunArmToAutoPositionCommand(DriveSubsystem robotDrive, boolean stopOnFinish) {
        return new FunctionalCommand(
                () -> {
                    double setpoint = MathUtils.angleRadiansToScoringTarget(robotDrive.getPose()) + armManualOffset;
                    double sanitizedSetpoint = sanitizePositionSetpoint(setpoint);
                    System.out.println("-----------------Starting Arm to position " + setpoint + ", Sanitized = " + sanitizedSetpoint + " --------------");
                    System.out.println("Setpoint = " + setpoint + ", offset = " + armManualOffset);
                    InitMotionProfile(sanitizedSetpoint);
                    manualControl = false;
                },
                () -> {
                    CalculateNewAutoAngle(robotDrive);
                },
                (interrupted) -> {
                    emergencyStop = false;
                },
                () -> {
                    return stopOnFinish && isFinished();
                }, this);
    }

    public Command RunArmUpManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Up Starting--------------");
                    manualControl = true;
                },
                () -> {
                    setSpeed(getSpeed.getAsDouble());
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return !isWithinLimits(getSpeed.getAsDouble());
                }, this);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Down Starting--------------");
                    manualControl = true;
                },
                () -> {
                    setSpeed(getSpeed.getAsDouble());
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return !isWithinLimits(getSpeed.getAsDouble());
                }, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
    }

}