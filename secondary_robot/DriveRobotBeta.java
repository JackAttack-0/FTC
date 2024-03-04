/* 
 * Modified version of TeleOp for secondary Robot
 * Copied from FTC/competition/DriveRobot.java on 1/7/24 16:39 PST
 */


package secondary_robot;
//https://www.youtube.com/watch?v=dQw4w9WgXcQ

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.List;
    
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="DriveRobotBeta", group ="Concept")
public class DriveRobotBeta extends LinearOpMode
{
    // Array to hold all 4 motors, this can be use in loops
    // such at this  " for(DcMotor motor : motors) {} " to execute 
    // action on all 4 motors.
    DcMotor    motors[]  = new DcMotor[4];

    DcMotor    frontLeftDrive   = null;
    DcMotor    frontRightDrive   = null;
    DcMotor    backLeftDrive   = null;
    DcMotor    backRightDrive   = null;
    IMU imu = null;
 
    void initRobot() {
        imu = hardwareMap.get(IMU.class, "imu");
        
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive  = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        motors[0]=(frontLeftDrive);
        motors[1]=(frontRightDrive);
        motors[2]=(backLeftDrive);
        motors[3]=(backRightDrive);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Move drone servo to loaded position
        loadDrone();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addData(">", "Press Start");
        telemetry.update();        
    }

    @Override public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        
        while (opModeIsActive())
        {
            double frontLeftDrivePower = 0;
            double frontRightDrivePower = 0;
            double backLeftDrivePower = 0;
            double backRightDrivePower = 0;
            
            double driveScale=-1;
            double sideScale=1;
            double turnScale=0.75;

            double driveInput = gamepad1.left_stick_y;
            double sideInput  = gamepad1.left_stick_x;
            double turnInput  = gamepad1.right_stick_x;

            frontLeftDrivePower = driveScale*driveInput
                          +sideScale*sideInput
                          +turnScale*turnInput;
            frontRightDrivePower = driveScale*driveInput
                          -sideScale*sideInput
                          -turnScale*turnInput;
            backLeftDrivePower = driveScale*driveInput
                          -sideScale*sideInput
                          +turnScale*turnInput;
            backRightDrivePower = driveScale*driveInput
                          +sideScale*sideInput
                          -turnScale*turnInput;
            double scale = 1;
            if (!gamepad1.right_bumper) {
                scale = 3.0;
            }
            
            scale=Math.abs(frontLeftDrivePower)>scale?Math.abs(frontLeftDrivePower):scale;
            scale=Math.abs(frontRightDrivePower)>scale?Math.abs(frontRightDrivePower):scale;
            scale=Math.abs(backLeftDrivePower)>scale?Math.abs(backLeftDrivePower):scale;
            scale=Math.abs(backRightDrivePower)>scale?Math.abs(backRightDrivePower):scale;

            frontLeftDrive.setPower(frontLeftDrivePower/scale);
            frontRightDrive.setPower(frontRightDrivePower/scale);
            backLeftDrive.setPower(backLeftDrivePower/scale);
            backRightDrive.setPower(backRightDrivePower/scale);
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addData("frontLeftDrive", frontLeftDrivePower/scale);
            telemetry.addData("frontRightDrive", frontRightDrivePower/scale);
            telemetry.addData("backLeftDrive", backLeftDrivePower/scale);
            telemetry.addData("backRightDrive", backRightDrivePower/scale);
            
            
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            
            telemetry.update();
            action="NONE";
            sleep(10);
        }
    }

    void turn(int angle) {
        imu.resetYaw();
        sleep(50);
        double currentAngle=getAngle();
        int direction=0;
        double targetAngle=currentAngle+angle;
        double remainingAngle=Math.abs(targetAngle-currentAngle);

        if(angle<0) {
            direction=-1;
        } else {
            direction=1;
        }

        while(remainingAngle>0.5) {

            double power=getTurnPower(remainingAngle);
            frontLeftDrive.setPower(-1*direction*power);
            frontRightDrive.setPower(1*direction*power);
            backLeftDrive.setPower(-1*direction*power);
            backRightDrive.setPower(1*direction*power);
            currentAngle=getAngle();

            // Adjust angle for angles greater than 180 or less than -180
            if(direction>0 && currentAngle<-10) {
                currentAngle=360+currentAngle;
            } else if(direction<0 && currentAngle>10) {
                currentAngle=-360+currentAngle;
            }
            
            remainingAngle=Math.abs(targetAngle-currentAngle);
        } 

        for(DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    double getTurnPower(double remainingAngle) {
        return((remainingAngle+10)/100);
    }

    void driveToDistance(int targetDistance) {
        double currentDistance=getDistance();
        double remainingDistance=currentDistance-targetDistance;

        while(remainingDistance>0) {

            double power=getDrivePower(remainingDistance);
            for(DcMotor motor : motors) {
                motor.setPower(power);
            }
            currentDistance=getDistance();
            remainingDistance=currentDistance-targetDistance;
        } 

        for(DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    double getDrivePower(double remainingDistance) {
        // What power to use to drive the robot
        final double DRIVE_POWER=0.6;
        // What power to use to drive the robot
        final double MIN_POWER=0.1;
        // Deceleration distance
        final double DECEL_DIST=800.0;

        if(remainingDistance>=DECEL_DIST) {
            return DRIVE_POWER;
        } else {
            return((DRIVE_POWER-MIN_POWER)*(remainingDistance/DECEL_DIST)+MIN_POWER);
        }   
    }

    double getAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }   
    int getDistanceL() {
        return (int)distanceSensorL.getDistance(DistanceUnit.MM);
    }   
    int getDistanceR() {
        return (int)distanceSensorR.getDistance(DistanceUnit.MM);
    }   

    boolean seeBlock(int distance) {
        if(getDistance()<=distance) {
            return true;
        } else {
            return false;
        }
    } 

    public void drive(int distance) {
        // Constants to use when driving the robot

        // To convert cm into motor position counter values
        final double DISTANCE_CONSTANT=2;
        // What power to use to drive the robot
        final double DRIVE_POWER=0.8;
        // What power to use to drive the robot
        final double MIN_POWER=0.1;
        // How long to pause before checking movement
        final int SLEEP_INTERVAL=10;
        // Acceleration distance (in encoder clicks). 300mm in this case:
        final double ACCEL_DIST=300.0*DISTANCE_CONSTANT;

        int targetPosition=(int)DISTANCE_CONSTANT*distance;

        for(DcMotor motor : motors) {
            // Stop and reset the motor counter
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Set the target position by converting the distance into motor
            // position values
            motor.setTargetPosition(targetPosition);       
            // Set the motor into the mode that uses the encoder to keep
            // track of the position
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 
        }
        
        telemetry.addData("frontLeftDrive",frontLeftDrive.getCurrentPosition());
        telemetry.update();

        // Sleep a bit to make sure the motor report as "busy"
        sleep(SLEEP_INTERVAL);
        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=frontLeftDrive.getCurrentPosition();
            telemetry.addData("frontLeftDrive", currentPosition);
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("distance", getDistance());
            telemetry.addData("distanceL", getDistanceL());
            telemetry.addData("distanceR", getDistanceR());
    
            // Determine the closest distance to either starting position
            // or target. When close to start, we accelerate, when close to 
            // target, we decelerate. When we are far from both, the robot 
            // drives at DRIVE_POWER speed. To avoid not moving at all, the
            // minimum speed is set to MIN_POWER. The distance over which to 
            // accerate or decelerate is ACCEL_DIST. All math is done in 
            // encoder "clicks", 300 mm is about 600 encoder clicks.
            int lengthToTarget=Math.abs(targetPosition-currentPosition);
            if (lengthToTarget>Math.abs(currentPosition)) {
                lengthToTarget=Math.abs(currentPosition);
            }
            
            double power=(DRIVE_POWER-MIN_POWER)*(lengthToTarget/ACCEL_DIST)+MIN_POWER;
            if(lengthToTarget>=ACCEL_DIST) {
                power=DRIVE_POWER;
            }
            
            for(DcMotor motor : motors) {
              motor.setPower(power);
            }
    
            // Sleep until next check
            sleep(SLEEP_INTERVAL);
            isBusy=false;
            for(DcMotor motor : motors) {
                if(motor.isBusy())isBusy=true;
            }
        } while(isBusy);

        for(DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }        
    } 


    public class testRunner {
        
        void square(int distance) {
            for (int i = 0; i < 4; i++) {
                drive(distance);
                turn(90);
            }
        }
    }

    public void strafe(int distance) {
        // Constants to use when driving the robot

        // To convert cm into motor position counter values
        final double DISTANCE_CONSTANT=2;
        // What power to use to drive the robot
        final double DRIVE_POWER=0.8;
        // What power to use to drive the robot
        final double MIN_POWER=0.1;
        // How long to pause before checking movement
        final int SLEEP_INTERVAL=10;
        // Acceleration distance (in encoder clicks). 300mm in this case:
        final double ACCEL_DIST=300.0*DISTANCE_CONSTANT;

        int targetPosition=(int)DISTANCE_CONSTANT*distance;
        int motorNumber=0;
        for(DcMotor motor : motors) {
            // Stop and reset the motor counter
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Set the motor into the mode that uses the encoder to keep
            // track of the position
            
            if(motorNumber==1 || motorNumber==2) {
                motor.setTargetPosition(targetPosition*-1);
            } else {
                motor.setTargetPosition(targetPosition);
            }
            
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set the target position by converting the distance into motor
            // position values
            

            motorNumber++;            
        }
        
        telemetry.addData("frontLeftDrive",frontLeftDrive.getCurrentPosition());
        telemetry.update();

        // Sleep a bit to make sure the motor report as "busy"
        sleep(SLEEP_INTERVAL);
        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=frontLeftDrive.getCurrentPosition();
            telemetry.addData("frontLeftDrive",currentPosition);
            telemetry.update();
    
            // Determine the closest distiance to either starting position
            // or target. When close to start, we accelerate, when close to 
            // target, we decelerate. When we are far from both, the robot 
            // drives at DRIVE_POWER speed. To avoid not moving at all, the
            // minimum speed is set to MIN_POWER. The distance over which to 
            // accerate or decelerate is ACCEL_DIST. All math is done in 
            // encoder "clicks", 300 mm is about 600 encoder clicks.
            int lengthToTarget=Math.abs(targetPosition-currentPosition);
            if (lengthToTarget>Math.abs(currentPosition)) {
                lengthToTarget=Math.abs(currentPosition);
            }
            
            double power=(DRIVE_POWER-MIN_POWER)*(lengthToTarget/ACCEL_DIST)+MIN_POWER;
            if(lengthToTarget>=ACCEL_DIST) {
                power=DRIVE_POWER;
            }
            
            for(DcMotor motor : motors) {
              motor.setPower(power);
            }
    
            // Sleep until next check
            sleep(SLEEP_INTERVAL);
            isBusy=false;
            for(DcMotor motor : motors) {
                if(motor.isBusy())isBusy=true;
            }
        } while(isBusy);

        for(DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }        
    } 
}