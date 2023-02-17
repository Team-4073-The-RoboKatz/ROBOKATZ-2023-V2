package frc.robot.Hardware;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TankDrive {
    
    


    private static CANSparkMax leftFrontMotor;
    private static CANSparkMax rightFrontMotor;

    public static double straight;

    public static void init() {
        leftFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
        //leftBackMotor = new CANSparkMax(LEFT_BACK_MOTOR_ID, MOTOR_TYPE);
        rightFrontMotor = new CANSparkMax(3, MotorType.kBrushless);
       // rightBackMotor = new CANSparkMax(RIGHT_BACK_MOTOR_ID, MOTOR_TYPE);

        rightFrontMotor.setInverted(true);

        //leftBackMotor.follow(leftFrontMotor);
        //rightBackMotor.follow(rightFrontMotor);
        

        leftFrontMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);
        //leftBackMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);
        rightFrontMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);
        //rightBackMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);
    }


    public void drive(double Speed, double turn) {
       
        leftFrontMotor.set(straight + turn);
        rightFrontMotor.set(straight - turn);
    }

    public static void driveTime(double leftSpeed, double rightSpeed) {
        leftFrontMotor.set(leftSpeed);
        rightFrontMotor.set(rightSpeed);
    }
}