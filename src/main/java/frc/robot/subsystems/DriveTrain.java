package frc.robot.subsystems;

import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    // private final DifferentialDriveOdometry odometry;
    // private final DifferentialDriveKinematics kinematics;
    private AnalogGyro gyro = new AnalogGyro(0); // DIO port
    // AutoBuilder.configure(
    //     this::getPose, // Robot pose supplier (returns the current robot pose)
    //     this::resetPose, // Method to reset odometry (called if your auto has a starting pose)
    //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. Must be robot-relative.
    //     (speeds, feedforwards) -> setAllMotorsSpeed(speeds, speeds), // Method that will drive the robot using ROBOT RELATIVE ChassisSpeeds
    //     PPLTVController(0.02), // Path following controller for differential drive
    //     config, // Robot configuration (e.g., max speeds, robot dimensions)
    //     () -> {
    //         // Boolean supplier that controls when the path will be mirrored for the red alliance
    //         // This will flip the path being followed to the red side of the field.
    //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE.
    //         DriverStation.Alliance alliance = DriverStation.getAlliance();
    //         return alliance == DriverStation.Alliance.Red; // Return true if it's the red alliance
    //     },
    //     this // Reference to this subsystem to set requirements
    // );
    
    // motors
    private final SparkMax leftFrontMotor;
    private final SparkMax leftRearMotor;
    private final SparkMax rightFrontMotor;
    private final SparkMax rightRearMotor;

    // motor configs
    private final SparkMaxConfig leftFrontConfig;
    private final SparkMaxConfig leftRearConfig;
    private final SparkMaxConfig rightFrontConfig;
    private final SparkMaxConfig rightRearConfig;

    // motor PID controllers
    private final SparkClosedLoopController leftFrontPID;
    private final SparkClosedLoopController leftRearPID;
    private final SparkClosedLoopController rightFrontPID;
    private final SparkClosedLoopController rightRearPID;

    // motor encoders
    private final RelativeEncoder leftFrontEncoder;
    private final RelativeEncoder leftRearEncoder;
    private final RelativeEncoder rightFrontEncoder;
    private final RelativeEncoder rightRearEncoder;

    // SmartVelocity PID (MaxMotion)
    private final double SmartVelocityP = 0.00011; // prev value: 0.0011
    private final double SmartVelocityI = 0;
    private final double SmartVelocityD = 0;
    private final double SmartVelocityFF = 0;

    // max motor speed
    // private final double MaxOutput = 1; // unused
    // min motor speed
    // private final double MinOutput = -1; // unused

    // max motor acceleration
    private final double maxAccel = 1000000000;
    // private final int SmartMotionID = 0; // unused
    // private int MaxMotionID = 1; // unused
    private final int maxVel = 4540;

    public final double allowedError = 0.05;

    private final DifferentialDrive differentialDrive;

    public DriveTrain() {

        // init motors
        leftFrontMotor = new SparkMax(Constants.leftFrontMotorPort, MotorType.kBrushless);
        leftRearMotor = new SparkMax(Constants.leftBackMotorPort, MotorType.kBrushless);
        rightFrontMotor = new SparkMax(Constants.rightFrontMotorPort, MotorType.kBrushless);
        rightRearMotor = new SparkMax(Constants.rightBackMotorPort, MotorType.kBrushless);

        // init config
        leftFrontConfig = new SparkMaxConfig();
        leftRearConfig = new SparkMaxConfig();
        rightFrontConfig = new SparkMaxConfig();
        rightRearConfig = new SparkMaxConfig();

        // Configuring the rear motors to follow the front motors
        leftRearConfig.follow(leftFrontMotor);
        rightRearConfig.follow(rightFrontMotor);

        // init PID controllers (closed loop)
        leftFrontPID = leftFrontMotor.getClosedLoopController();
        leftRearPID = leftRearMotor.getClosedLoopController();
        rightFrontPID = rightFrontMotor.getClosedLoopController();
        rightRearPID = rightRearMotor.getClosedLoopController();

        // init encoders
        leftFrontEncoder = leftFrontMotor.getEncoder();
        leftRearEncoder = leftRearMotor.getEncoder();
        rightFrontEncoder = rightFrontMotor.getEncoder();
        rightRearEncoder = rightRearMotor.getEncoder();

        // set idle mode for motors
        leftFrontConfig.idleMode(IdleMode.kBrake);
        leftRearConfig.idleMode(IdleMode.kBrake);
        rightFrontConfig.idleMode(IdleMode.kBrake);
        rightRearConfig.idleMode(IdleMode.kBrake);

        leftFrontConfig.inverted(false);
        // leftRear.setInverted(false);
        rightFrontConfig.inverted(true);
        // rightRear.setInverted(true);

        // set voltage compensation
        leftFrontConfig.voltageCompensation(12.0);
        leftRearConfig.voltageCompensation(12.0);
        rightFrontConfig.voltageCompensation(12.0);
        rightRearConfig.voltageCompensation(12.0);

        // Set current limits
        leftFrontConfig.smartCurrentLimit(40);
        leftRearConfig.smartCurrentLimit(40);
        rightFrontConfig.smartCurrentLimit(40);
        rightRearConfig.smartCurrentLimit(40);

        // configure PID controllers
        configurePIDControllers();

        // Reset encoders to start at zero
        leftFrontEncoder.setPosition(0);
        rightFrontEncoder.setPosition(0);


        // restore factory defaults and set new configs for each motor
        leftFrontConfig.signals.primaryEncoderPositionPeriodMs(5);
        leftFrontMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftRearConfig.signals.primaryEncoderPositionPeriodMs(5);
        leftRearMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightFrontConfig.signals.primaryEncoderPositionPeriodMs(5);
        rightFrontMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightRearConfig.signals.primaryEncoderPositionPeriodMs(5);
        rightRearMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // differential drive init
        differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    }

    private void configurePIDControllers() {
        // Apply PID constants to the PID controllers
        // left front motor
        leftFrontConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftFrontConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        leftFrontConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        leftFrontConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        leftFrontConfig.closedLoop.velocityFF(SmartVelocityFF);
        leftFrontConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);
        // leftFrontConfig.closedLoop.outputRange(MinOutput, MaxOutput);

        // leftFrontPID.setOutputRange(MinOutput, MaxOutput, MaxMotionID);
        // leftFrontConfig.setSmartMotionMinOutputVelocity(minVel, MaxMotionID);

        // left rear motor
        leftRearConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftRearConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        leftRearConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        leftRearConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        leftRearConfig.closedLoop.velocityFF(SmartVelocityFF);
        leftRearConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);

        // leftRearPID.setOutputRange(MinOutput, MaxOutput, MaxMotionID);
        // leftRearPID.setSmartMotionMinOutputVelocity(minVel, MaxMotionID);

        // right front motor
        rightFrontConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightFrontConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        rightFrontConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        rightFrontConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        rightFrontConfig.closedLoop.velocityFF(SmartVelocityFF);
        rightFrontConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);

        // rightFrontPID.setOutputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot1);
        // rightFrontPID.setSmartMotionMinOutputVelocity(minVel, ClosedLoopSlot.kSlot1);

        // right rear motor
        rightRearConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightRearConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        rightRearConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        rightRearConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        rightRearConfig.closedLoop.velocityFF(SmartVelocityFF);
        rightRearConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);

        // rightRearPID.setOutputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot1);
        // rightRearPID.setSmartMotionMinOutputVelocity(minVel, ClosedLoopSlot.kSlot1);
    }

    /**
     * @return rotational value of the motor
     */
    public double getleftFrontEncoder() {
        return leftFrontEncoder.getPosition();
    }

    /**
     * @return rotational value of the motor
     */
    public double getleftRearEncoder() {
        return leftRearEncoder.getPosition();
    }

    /**
     * @return rotational value of the motor
     */
    public double getRightFrontEncoder() {
        return rightFrontEncoder.getPosition();
    }

    /**
     * @return rotational value of the motor
     */
    public double getRightRearEncoder() {
        return rightRearEncoder.getPosition();
    }

    /**
     * @param input percent input of speed from left joystick
     */
    public void setLeftSideMotorSpeed(double input) {
        // calculate the necessary rpm
        double speed = input * maxVel;
        SmartDashboard.putNumber("Speed of left side(rpm): ", speed);
        System.out.println("right speed: " + speed);
        // Set rpm for left motors
        leftFrontPID.setReference(speed, ControlType.kMAXMotionVelocityControl);
        leftRearPID.setReference(speed, ControlType.kMAXMotionVelocityControl);
        // leftFrontMotor.set(speed);
        // leftRearMotor.set(speed);
    }

    /**
     * @param input percent input of speed from right joystick
     */
    public void setRightSideMotorSpeed(double input) {
        // calculate the necessary rpm
        double speed = input * maxVel;
        SmartDashboard.putNumber("Speed of right side(rpm): ", speed);
        System.out.println("right speed: " + speed);
        // Set rpm for left motors
        rightFrontPID.setReference(-speed, ControlType.kMAXMotionVelocityControl);
        rightRearPID.setReference(-speed, ControlType.kMAXMotionVelocityControl);
        // rightFrontMotor.set(speed);
        // rightRearMotor.set(speed);
    }

    /**
     * @param speed percent output for both motors
     */
    public void setAllMotorsSpeed(double leftSpeed, double rightSpeed) {
        setLeftSideMotorSpeed(leftSpeed);
        setRightSideMotorSpeed(rightSpeed);
    }

    /**
     * @param rotations percent output for robot
     */
    public void setLeftSideMotorsPosition(double rotations) {
        // Controlling robot through percent output/motion control
        // System.out.println("rpm: " + rotations);
        leftFrontPID.setReference(rotations, ControlType.kDutyCycle);
        leftRearPID.setReference(rotations, ControlType.kDutyCycle);
    }

    /**
     * @param rotations percent output for robot
     */
    public void setRightSideMotorsPosition(double rotations) {
        // Controlling robot through percent output/motion control
        // System.out.println("rpm: " + rotations);
        rightFrontPID.setReference(rotations, ControlType.kDutyCycle);
        rightRearPID.setReference(rotations, ControlType.kDutyCycle);
    }

    /**
     * @param rotations percent output for robot motors
     */
    public void setAllMotorsPosition(double rotations) {
        setLeftSideMotorsPosition(rotations);
        setRightSideMotorsPosition(rotations);
    }

    public void resetLeftSideEncoders() {
        // Set encoder positon as 0
        leftFrontEncoder.setPosition(0);
        leftRearEncoder.setPosition(0);
    }

    public void resetRightSideEncoders() {
        // Set encoder positon as 0
        rightFrontEncoder.setPosition(0);
        rightRearEncoder.setPosition(0);
    }

    public void resetAllEncoders() {
        resetLeftSideEncoders();
        resetRightSideEncoders();
    }

    /**
     * @param leftSpeed percent output of left
     * @param rightSpeed percent output of right
     */
    public void arcadeDrive(double leftSpeed, double rightSpeed) {
        // Only use for practice training
        // differentialDrive.arcadeDrive(leftSpeed, rightSpeed);
    }

    // Odometry Method - Get current robot pose
    // public Pose2d getPose() {
    //     return odometry.getPoseMeters();
    // }

    // // Reset Odometry Method - Reset to a given pose (typically from your autonomous routine)
    // public void resetPose(Pose2d pose) {
    //     odometry.resetPosition(gyro.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition(), pose);
    // }

    // // Get Robot Relative Speeds for PathPlanner (used by AutoBuilder)
    // public ChassisSpeeds getRobotRelativeSpeeds() {
    //     double leftSpeed = leftFrontEncoder.getVelocity();  // Left motor speed (meters per second)
    //     double rightSpeed = rightFrontEncoder.getVelocity();  // Right motor speed (meters per second)
    //     return new ChassisSpeeds(leftSpeed, 0, rightSpeed);  // Tank drive has no strafe speed, so Y is 0
    // }

    // // Gyro Yaw - Returns the robot's heading (rotation) for odometry
    // public Rotation2d getGyroYaw() {
    //     return gyro.getRotation2d();  // Assumes Gyro object gives rotation in degrees or radians
    // }

    private double getHeading() {
        // Get the current gyro angle (assuming counterclockwise is positive)
        return Math.IEEEremainder(gyro.getAngle(), 360); // Keeps angle in range [-180, 180]
    }

    @Override
    public void periodic() {
        // Update odometry, smart dashboard, etc.
        differentialDrive.feed();
        // Update the odometry with encoder positions and gyro heading
        // odometry.update(getGyroYaw(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

        // Optionally, display the robot's current position on the SmartDashboard
        // SmartDashboard.putNumber("Robot X", odometry.getPoseMeters().getX());
        // SmartDashboard.putNumber("Robot Y", odometry.getPoseMeters().getY());
        // SmartDashboard.putNumber("Robot Heading", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("angle: ", gyro.getAngle());

        // System.out.println(4 * (dutyCycle.getOutput() - 1000));
    }
}