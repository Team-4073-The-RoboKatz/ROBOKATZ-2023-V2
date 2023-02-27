// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final XboxController c_xbox1 = new XboxController(0);
  private final XboxController c_xbox2 = new XboxController(1);

    public CANSparkMax m_LFrontMotor;
    public CANSparkMax  m_LBackMotor;
    public CANSparkMax m_RFrontMotor;
    public CANSparkMax  m_RBackMotor; 
    TalonSRX arm  = new TalonSRX(6);
    TalonSRX claw = new TalonSRX(7);
  {
    m_LFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_LBackMotor  = new CANSparkMax(3, MotorType.kBrushless);
    m_RFrontMotor = new CANSparkMax(4, MotorType.kBrushless);
    m_RBackMotor  = new CANSparkMax(5, MotorType.kBrushless);
  }
  MotorControllerGroup m_left  = new MotorControllerGroup(m_LFrontMotor, m_LBackMotor);
  MotorControllerGroup m_right = new MotorControllerGroup(m_RFrontMotor, m_RBackMotor);
  double  strait;
  double    turn;
  boolean toggle;
  SlewRateLimiter filter = new SlewRateLimiter(0.2); 

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
{
  m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
}
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(4);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("cam1");

  // PID constants should be tuned per robot
  final double LINEAR_P = -0.02;
  final double LINEAR_D = 0.01;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = -0.02;
  final double ANGULAR_D = 0.01;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
        
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double filteredrot;
    double filterfow;
    double forwardSpeed;
    double rotationSpeed;
    toggle = false; 


     if (c_xbox1.getXButtonPressed()) {
      if (toggle) { 
        SmartDashboard.putString("Lights", "Off");
         // Current state is true so turn off
         lightsoff();
         toggle = false;
      } else {
        SmartDashboard.putString("Lights", "On");
         // Current state is false so turn on
         lightson();
         toggle = true;
      }
      if (c_xbox1.getYButtonPressed()){
        camera.setPipelineIndex(1);
        System.out.println(camera.getPipelineIndex());
      }
      else{
       camera.setPipelineIndex(0);
       System.out.println(camera.getPipelineIndex());
      }
    }
    if (c_xbox1.getAButton()) {
      camera.setPipelineIndex(0);
        // Vision-alignment mode
        System.out.println("Finding RR Targets");
        // Query the latest result from PhotonVision
        var RRresult = camera.getLatestResult();
        if (RRresult.hasTargets()) {
          System.out.println("Reflective Target Found");
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(RRresult.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(RRresult.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            System.out.println("No RR Targets");
            forwardSpeed = 0;
            rotationSpeed = 0;
            System.out.println("Finding April Targets");
            camera.setPipelineIndex(1);
        // Query the latest result from PhotonVision
        var ATresult = camera.getLatestResult();
        if (ATresult.hasTargets()) {
          System.out.println("April Target Found");
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(ATresult.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(ATresult.getBestTarget().getYaw(), 0);
    } else {
        // Manual Driver Mode
        
        forwardSpeed = -c_xbox1.getLeftY();
        rotationSpeed = c_xbox1.getLeftTriggerAxis() - c_xbox1.getRightTriggerAxis();
    }
    // Use our forward/turn speeds to control the drivetrain
    filteredrot = filter.calculate(rotationSpeed);
    filterfow = filter.calculate(forwardSpeed);
    m_robotDrive.arcadeDrive(filterfow, filteredrot);
  }
}


    if(c_xbox2.getRightBumperPressed()){
      arm.set(ControlMode.PercentOutput, .5);
    }else{
      arm.set(ControlMode.PercentOutput, 0);;}
    if(c_xbox2.getLeftBumperPressed()){
      arm.set(ControlMode.PercentOutput, -.5);
    }else{
      arm.set(ControlMode.PercentOutput, 0);
    }
}
    
 private void lightson() {
  camera.setLED(VisionLEDMode.kOn);
  }

private void lightsoff() {
  camera.setLED(VisionLEDMode.kOff);
  }

/** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

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
