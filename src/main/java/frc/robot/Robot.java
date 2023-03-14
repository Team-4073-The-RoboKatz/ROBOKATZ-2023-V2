// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "Better Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  DoubleSolenoid armSolenoid = new DoubleSolenoid(12, PneumaticsModuleType.REVPH, 0,1);

  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  private final XboxController c_xbox1 = new XboxController(0);
  private final XboxController c_xbox2 = new XboxController(1);
    TalonSRX armSRX = new TalonSRX(6);
   
    VictorSPX m_intakeSPX1 = new VictorSPX(8);
    VictorSPX m_intakeSPX2 = new VictorSPX(7);
    public CANSparkMax m_LFrontMotor;
    public CANSparkMax  m_LBackMotor;
    public CANSparkMax m_RFrontMotor;
    public CANSparkMax  m_RBackMotor; 
    private final Timer m_timer =new Timer();
  {
    m_LFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_LBackMotor  = new CANSparkMax(3, MotorType.kBrushless);
    m_RFrontMotor = new CANSparkMax(4, MotorType.kBrushless);
    m_RBackMotor  = new CANSparkMax(5, MotorType.kBrushless);

    m_LFrontMotor.setIdleMode(IdleMode.kBrake);
    m_LBackMotor. setIdleMode(IdleMode.kBrake);
    m_RFrontMotor.setIdleMode(IdleMode.kBrake);
    m_RBackMotor. setIdleMode(IdleMode.kBrake);
    
    m_RBackMotor. setInverted(true);
    m_RFrontMotor.setInverted(true);

    m_intakeSPX2.setInverted(true);
    armSRX.setNeutralMode(NeutralMode.Brake);

  }
  MotorControllerGroup m_left  = new MotorControllerGroup(m_LFrontMotor, m_LBackMotor);
  MotorControllerGroup m_right = new MotorControllerGroup(m_RFrontMotor, m_RBackMotor);


  double strait;
  double turn;


  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("cam1");
   
  NetworkTableEntry      rawBytes;
  NetworkTableEntry     hasTarget;
  NetworkTableEntry latencyMillis;
  NetworkTableEntry    targetpose;
  NetworkTableEntry     targetYaw;
  NetworkTableEntry pipelineIndex;
  NetworkTableEntry       ledMode; 
    
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private final AutoBalancer autoBalancer = new AutoBalancer(imu, m_robotDrive);

public void togglearm(){
  Value oppValue = armSolenoid.get() == Value.kForward ? Value.kReverse : Value.kForward;
  armSolenoid.set(oppValue);
}
{
  m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
}
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(2);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(2);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("cam1");

  // PID constants should be tuned per robot
  final double LINEAR_P = -0.1;
  final double LINEAR_D = 0.01;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = -0.1;
  final double ANGULAR_D = 0.01;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
 
  // Using the default constructor of RamseteController. Here
// the gains are initialized to 2.0 and 0.7.
RamseteController controller1 = new RamseteController();

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    imu.reset();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    if (m_timer.get() <2.2) {
      m_robotDrive.arcadeDrive(0.6, 0);
     }else{
      m_robotDrive.stopMotor();
     }
  }
    

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    imu.reset();
    imu.calibrate();
    
    double armvar = c_xbox2.getRightTriggerAxis() - c_xbox2.getLeftTriggerAxis();

    m_robotDrive.arcadeDrive(strait, turn);
    double forwardSpeed;
    double rotationSpeed;
    int index = camera.getPipelineIndex();
    
    //control the arm rotation
    armSRX.set(ControlMode.PercentOutput, armvar);
    //control the arm extention
    if(c_xbox2.getAButton()){
      togglearm();
    }
    //control pipeline index
    if (c_xbox1.getXButtonPressed()){
      index++;
    camera.setPipelineIndex(index);
    }
    if (c_xbox2.getXButtonPressed() || index >=2){
      camera.setPipelineIndex(0);
    }

//Vison
    if (c_xbox1.getAButton()) {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            forwardSpeed = 0;
            rotationSpeed = 0;
        }
    } else {
        // Manual Driver Mode
        forwardSpeed = -c_xbox1.getLeftY();
        rotationSpeed = (c_xbox1.getLeftTriggerAxis() - c_xbox1.getRightTriggerAxis())/1.25;    
      }

    // Use our forward/turn speeds to control the drivetrain
    m_robotDrive.arcadeDrive(forwardSpeed, rotationSpeed);
  
//balance
if(c_xbox1.getYButtonPressed()){
  autoBalancer.autoBalance();
}
  }
    
  public class AutoBalancer {
    private final ADIS16470_IMU imu;
    private final DifferentialDrive robotDrive;
    private final double kP = 1;
    private final double kI = 1.00;
    private final double kD = 1;
    private double previousError = 0.0;
    private double integral = 0.0;
    
    public AutoBalancer(ADIS16470_IMU imu, DifferentialDrive robotDrive) {
        this.imu = imu;
        this.robotDrive = robotDrive;
    }
    
    public void autoBalance() {
        double angle = imu.getAccelZ();
        double error = -angle;
        integral += error;
        double derivative = error - previousError;
        double output = kP * error + kI * integral + kD * derivative;
        robotDrive.tankDrive(-output, -output);
        previousError = error;
    }
  }
 /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
