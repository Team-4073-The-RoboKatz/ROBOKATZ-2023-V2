/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto1 = "Custom Auto 1";
  private static final String kCustomAuto2 = "Custom Auto 2";
  private static final String kCustomAuto3 = "Custom Auto 3";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  //creates SPARK MAX motor controller devices
  private DifferentialDrive m_Drive;
  private static final int LFrontDeviceID = 2; 
  private static final int LBackDeviceID = 3;
  private static final int RFrontDeviceID =4;
  private static final int RBackDeviceID = 5;
  private CANSparkMax m_LFrontMotor;
  private CANSparkMax m_LBackMotor;
  private CANSparkMax m_RFrontMotor;
  private CANSparkMax m_RBackMotor;
  //controllers
  XboxController c_xbox1 = new XboxController(0);
  XboxController c_xbox2 = new XboxController(1);

  //Vision Data stuff
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("cam1");
   
  NetworkTableEntry rawBytes;
  NetworkTableEntry hasTarget;
  NetworkTableEntry latencyMillis;
  NetworkTableEntry targetpose;
  NetworkTableEntry targetYaw;
  NetworkTableEntry pipelineIndex;
  NetworkTableEntry ledMode; 

  //variables
  public double straight;
  public double turn;
  public double maxspeed =(.5);
  public double arm;
  public double arm_var;
  public double leftSpeed = straight+turn;
  public double rightSpeed =straight-turn;
  public double forwardSpeed;
  public double rotationSpeed;
  boolean toggleOn = false;
  boolean togglePressed = false;
  
  
  //Pneumatics
   Solenoid example1SolenoidPCM = new Solenoid(PneumaticsModuleType.REVPH, 1);
   Solenoid example2SolenoidPCM = new Solenoid(PneumaticsModuleType.REVPH, 2);

   //PHOTONVISION INIT 
       //TODO Constants such as camera and target height stored. Change per robot and goal!
       final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
       final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
       // Angle between horizontal and the camera.
       final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
   
       // How far from the target we want to be
       final double GOAL_RANGE_METERS = Units.feetToMeters(3);
   
       // Change this to match the name of your camera
       PhotonCamera camera = new PhotonCamera("cam1");
   
       // PID constants should be tuned per robot
       final double LINEAR_P = 0.1;
       final double LINEAR_D = 0.0;
       PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
   
       final double ANGULAR_P = 0.1;
       final double ANGULAR_D = 0.0;
       PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);


  @Override
  public void robotInit() {
  m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Auto 1", kCustomAuto1);
    m_chooser.addOption("Auto 2", kCustomAuto2);
    m_chooser.addOption("Auto 3", kCustomAuto3);
    SmartDashboard.putData("Auto choices", m_chooser);

  //SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object

    m_LFrontMotor = new CANSparkMax(LFrontDeviceID, MotorType.kBrushless);
    m_LBackMotor = new CANSparkMax(LBackDeviceID, MotorType.kBrushless);
    m_RFrontMotor = new CANSparkMax(RFrontDeviceID, MotorType.kBrushless);
    m_RBackMotor = new CANSparkMax(RBackDeviceID, MotorType.kBrushless);
  

    m_LFrontMotor.restoreFactoryDefaults();
    m_LBackMotor.restoreFactoryDefaults();
    m_RFrontMotor.restoreFactoryDefaults();
    m_RBackMotor.restoreFactoryDefaults();
    //set right side inverted
    m_RFrontMotor.setInverted(true);
    m_RBackMotor.setInverted(true);
    //set the motors to brake on idle.
    m_LFrontMotor.setIdleMode(IdleMode.kBrake);
    m_LBackMotor.setIdleMode(IdleMode.kBrake);
    m_RFrontMotor.setIdleMode(IdleMode.kBrake);
    m_RBackMotor.setIdleMode(IdleMode.kBrake);
    // Run after all parameters are set in RobotInit()
    m_LFrontMotor.burnFlash();
    m_LBackMotor.burnFlash();
    m_RFrontMotor.burnFlash();
    m_RBackMotor.burnFlash();
    //create motor controller groups
    MotorControllerGroup mg_leftGroup = new MotorControllerGroup(m_LFrontMotor, m_LBackMotor);
    MotorControllerGroup mg_rightGroup = new MotorControllerGroup(m_RFrontMotor, m_RBackMotor);

    m_Drive = new DifferentialDrive(mg_leftGroup, mg_rightGroup);
    m_LFrontMotor.getEncoder();
    m_LBackMotor.getEncoder();
    m_RFrontMotor.getEncoder();
    m_RBackMotor.getEncoder();
    //SET PhotonVision PipeLine index to 0, 0 = Retroreflective 
    camera.setPipelineIndex(0);
    //Turn of LL LEDs.. 
    camera.setLED(VisionLEDMode.kOff);

  }@Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void teleopPeriodic() {


    if (Math.abs(straight) < 0.1) {
      straight = 0;
    {
  
    if (Math.abs(turn) < 0.1) {                                                    
      turn = 0;
    }
  
    turn =  c_xbox1.getRawAxis(3) - c_xbox1.getRawAxis(2);
    straight = -c_xbox1.getLeftY();

    //PhotonVision
    double forwardSpeed;
    double rotationSpeed;

    if (c_xbox1.getAButton()) { 
      camera.setLED(VisionLEDMode.kBlink);
        try {
          wait(100);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
        camera.setLED(VisionLEDMode.kOn);
        try {
          wait(500);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      // Vision-alignment mode

      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();
      System.out.println("Finding Targets");
      if (result.hasTargets()) {
        System.out.println("Targets Found");
          // First calculate range
          double range =
                  PhotonUtils.calculateDistanceToTargetMeters(
                          CAMERA_HEIGHT_METERS,
                          TARGET_HEIGHT_METERS,
                          CAMERA_PITCH_RADIANS,
                          Units.degreesToRadians(result.getBestTarget().getPitch()));

          // Use this range as the measurement we give to the PID controller.
          // -1.0 required to ensure positive PID controller effort _increases_ range
          forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

          // Also calculate angular power
          // -1.0 required to ensure positive PID controller effort _increases_ yaw
          rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      } else {
          // If we have no targets, stay still.
          System.out.println("Could Not Find Target");
    
          forwardSpeed = 0;
          rotationSpeed = 0;
          camera.setLED(VisionLEDMode.kOff);
      }
  } else {
      // Manual Driver Mode
      forwardSpeed = -c_xbox1.getLeftY();
      rotationSpeed = turn;

  // Use our forward/turn speeds to control the drivetrain
  m_Drive.arcadeDrive(forwardSpeed, rotationSpeed); 
 
   //math to help eleminate stick drift

  
} 
    if(c_xbox2.getBButton()) {
    example1SolenoidPCM.set(true);
   } else {
    example1SolenoidPCM.set(false);
   } 
   
    if(c_xbox2.getXButton()) {
    example2SolenoidPCM.set(true);
   } else {
    example1SolenoidPCM.set(false);
  };

        
        }
}
}
}