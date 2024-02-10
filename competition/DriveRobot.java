package org.firstinspires.ftc.teamcode;
//https://www.youtube.com/watch?v=dQw4w9WgXcQ

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;
import java.util.List;

@TeleOp(name="DriveRobot", group ="Concept")
public class DriveRobot extends LinearOpMode
{
    // Array to hold all 4 motors, this can be use in loops
    // such at this  " for(DcMotor motor : motors) {} " to execute 
    // action on all 4 motors.
    DcMotor    motors[]  = new DcMotor[4];

    DcMotor    motor1   = null;
    DcMotor    motor2   = null;
    DcMotor    motor3   = null;
    DcMotor    motor4   = null;
    DcMotor    hangerL  = null;
    DcMotor    hangerR  = null;
    Servo      launcher = null;
    Servo      claw     = null;
    Servo      wrist    = null;
    CRServo    scissorL = null;
    CRServo    scissorR = null;
    DcMotor    liftR    = null;
    DcMotor    liftL    = null;
    boolean    isLiftMoving = false;
    boolean    isHangerMoving = false;
    boolean    isPlaneLaunched = false;  
    boolean    wristUp=false;
    boolean    wristHigh=false;
    boolean    clawClosed=false;
    String     wristStatus="void";
    String     clawStatus="void";
    String     hangingStatus="void";
    String     action="void";
    DistanceSensor distanceSensor = null;
    DistanceSensor distanceSensorL = null;
    DistanceSensor distanceSensorR = null;
    IMU imu = null;
 
    void initRobot() {
        imu = hardwareMap.get(IMU.class, "imu");
        
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        motor3  = hardwareMap.get(DcMotor.class, "motor3");
        motor4  = hardwareMap.get(DcMotor.class, "motor4");
        hangerL  = hardwareMap.get(DcMotor.class, "hangerL");
        hangerR  = hardwareMap.get(DcMotor.class, "hangerR");
        motors[0]=(motor1);
        motors[1]=(motor2);
        motors[2]=(motor3);
        motors[3]=(motor4);

        launcher = hardwareMap.get(Servo.class, "launcher");
        claw    = hardwareMap.get(Servo.class, "claw");
      
        wrist    = hardwareMap.get(Servo.class, "wrist");

        scissorL   = hardwareMap.get(CRServo.class, "scissorL");
        scissorR   = hardwareMap.get(CRServo.class, "scissorR");
        
        liftR    = hardwareMap.get(DcMotor.class, "liftR");
        liftL    = hardwareMap.get(DcMotor.class, "liftL");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensorL = hardwareMap.get(DistanceSensor.class, "distanceSensorL");
        distanceSensorR = hardwareMap.get(DistanceSensor.class, "distanceSensorR");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        // Move drone servo to loaded position
        //loadDrone();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addData(">", "Press Start");
        telemetry.update();        
    }

    @Override public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        
        while (opModeIsActive())
        {
            double motor1Power = 0;
            double motor2Power = 0;
            double motor3Power = 0;
            double motor4Power = 0;
            
            double driveScale=-1;
            double sideScale=1;
            double turnScale=0.75;

            double driveInput = gamepad1.left_stick_y;
            double sideInput  = gamepad1.left_stick_x;
            double turnInput  = gamepad1.right_stick_x;

            motor1Power = driveScale*driveInput
                          +sideScale*sideInput
                          +turnScale*turnInput;
            motor2Power = driveScale*driveInput
                          -sideScale*sideInput
                          -turnScale*turnInput;
            motor3Power = driveScale*driveInput
                          -sideScale*sideInput
                          +turnScale*turnInput;
            motor4Power = driveScale*driveInput
                          +sideScale*sideInput
                          -turnScale*turnInput;
            double scale = 3;
            if (gamepad1.right_bumper) {
                scale = 1.5;
            }
            if (gamepad1.left_bumper) {
                scale = 1.0;
            }
            
            scale=Math.abs(motor1Power)>scale?Math.abs(motor1Power):scale;
            scale=Math.abs(motor2Power)>scale?Math.abs(motor2Power):scale;
            scale=Math.abs(motor3Power)>scale?Math.abs(motor3Power):scale;
            scale=Math.abs(motor4Power)>scale?Math.abs(motor4Power):scale;

            motor1.setPower(motor1Power/scale);
            motor2.setPower(motor2Power/scale);
            motor3.setPower(motor3Power/scale);
            motor4.setPower(motor4Power/scale);
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            
            telemetry.addData("!","==HEADS_UP==");
            telemetry.addData("","");
            
            telemetry.addData("ACTION", action);
            telemetry.addData("HEADING", "%.2f Deg.", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("WRIST", wristStatus);
            telemetry.addData("CLAW", clawStatus);
            telemetry.addData("HANGING", hangingStatus);
            
            telemetry.addData("","");
            telemetry.addData("!","==DIAGNOSTICS==");
            telemetry.addData("","");

            telemetry.addData("motor1", motor1Power/scale);
            telemetry.addData("motor2", motor2Power/scale);
            telemetry.addData("motor3", motor3Power/scale);
            telemetry.addData("motor4", motor4Power/scale);
            
            
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("claw", Math.round(claw.getPosition()*1000) + " (" + clawStatus + ")");
            telemetry.addData("wrist", Math.round(wrist.getPosition()*1000) + " (" + wristStatus + ")");
            telemetry.addData("distance", getDistance());
            telemetry.addData("distanceL", getDistanceL());
            telemetry.addData("distanceR", getDistanceR());

            if(gamepad1.dpad_right) {
                if(gamepad1.right_bumper) {
                    turn(-15);
                } else {
                    turn(-90);
                }
            }

            if(gamepad1.dpad_left) {
                if(gamepad1.right_bumper) {
                    turn(15);
                } else {
                    turn(90);
                }
            }
            

            if(gamepad2.dpad_up){
                launchDrone();
                sleep(1000);
                //loadDrone();
                action="FIRING DRONE";
            }

            if(gamepad1.a){
                hangerDown();
            }
            if(gamepad1.b){
                hangerRaise();
            }

            if (gamepad1.dpad_down) {
                turn(180);
                sleep(200);
            }

            if(gamepad1.dpad_up){
                extendLift();
                sleep(1000);
                stopLift();
                driveToDistance(300);
                wristDown();
                release();
                drive(-70, 1);
                contractLift();
                sleep(1000);
                stopLift();
            }

            if(gamepad2.b) {
                if(clawClosed) {
                    release();
                    clawClosed=false;
                    clawStatus="Claw Open";
                    action="OPEN CLAW";
                } else {
                    grab();
                    clawClosed=true;
                    clawStatus="Claw Holding";
                    action="CLOSE CLAW";
                }
                sleep(500);
            }
            
            if(gamepad2.y) {
                if(Math.floor(wrist.getPosition()*1000) == 360) {
                    wristDown();
                    wristStatus="Wrist to Ground";
                    action="WRIST GROUNDED";
                } else {
                    wristUp();
                    wristStatus="Wrist to Board Angle";
                    action="WRIST TO BOARD";
                }
                sleep(500);
            }
            
            if(gamepad2.a) {
                liftPixel();
            }
            
            if(gamepad2.right_bumper) {
                raiseHooks(gamepad2.dpad_left,gamepad2.dpad_right);
            }
            
            if(gamepad2.left_bumper) {
                lowerHooks(gamepad2.dpad_left,gamepad2.dpad_right);
            }
            
            if(gamepad2.x) {
                if(!(Math.floor(wrist.getPosition()*1000) == 0)) {
                    wristHigh();
                    wristStatus="Wrist Up";
                    action="WRIST UP";
                } else {
                    wristUp();
                    wristStatus="Wrist to Board Angle";
                    action="WRIST TO BOARD";
                }
                sleep(500);
            }
            
            if(gamepad2.right_trigger>0.1) {
                extendLift();
                action="LIFT EXPAND";
            }

            if(gamepad2.left_trigger>0.1) {
                contractLift();
                action="LIFT CONTRACT";
            }

            if(isLiftMoving &&    
               gamepad2.left_trigger<=0.1 &&
               gamepad2.right_trigger<=0.1) {
               stopLift();
               action="LIFT STOP";
            }

            if(isHangerMoving &&
            !gamepad1.a &&
            !gamepad1.b) {
               stopHanger();
            }

            telemetry.update();
            action="NONE";
            sleep(10);
        }
    }

    void release() {
        claw.setPosition(0.5);
        
    }

    void grab() {
        claw.setPosition(1);
        
    }

    void wristUp() {
        wrist.setPosition(0.36);
    }

    void wristDown() {
        wrist.setPosition(0.625);
    }
    
    void wristHigh() {
        wrist.setPosition(0);
    }
    
    void raiseHooks(boolean left, boolean right) {
        scissorL.setPower((left) ? 1 : 0);
        scissorR.setPower((right) ? -1 : 0);
        scissorL.setPower((!left && !right) ? 1 : 0);
        scissorR.setPower((!left && !right) ? -1 : 0);
        sleep(200);
        scissorL.setPower(0);
        scissorR.setPower(0);
        
    }
    
    void lowerHooks(boolean left, boolean right) {
        scissorL.setPower((left) ? -1 : 0);
        scissorR.setPower((right) ? 1 : 0);
        scissorL.setPower((!left && !right) ? -1 : 0);
        scissorR.setPower((!left && !right) ? 1 : 0);
        sleep(200);
        scissorL.setPower(0);
        scissorR.setPower(0);
        
    }

    void loadDrone() {
        launcher.setPosition(0.35);
    }

    void launchDrone() {
        launcher.setPosition(0.25);
        isPlaneLaunched = true;
    }

    void hangerDown() {
        hangerL.setPower(0.3);
        hangerR.setPower(-0.3);
        isHangerMoving = true; 
    }

    void hangerRaise() {
        hangerL.setPower(-.75);
        hangerR.setPower(.75);
        isHangerMoving = true; 

    }

     void stopHanger() {
        hangerL.setPower(0);
        hangerR.setPower(0);
        isHangerMoving = false;
    }


    void extendLift() {
        liftL.setPower(.65);
        liftR.setPower(-.65);
        isLiftMoving = true; 
    }

    void contractLift() {
        liftL.setPower(-.65);
        liftR.setPower(.65);
        isLiftMoving = true;
    }

    void stopLift() {
        liftL.setPower(0);
        liftR.setPower(0);
        hangerL.setPower(0);
        hangerR.setPower(0);
        isLiftMoving = false;
    }

    void liftPixel() {
        drive(-110,0);
        sleep(1000);
        release();
        sleep(1000);
        wristDown();
        extendLift();
        sleep(100);
        stopLift();
        sleep(1000);
        grab();
        sleep(1000);
        extendLift();
        sleep(750);
        stopLift();
        sleep(1000);
        wristUp();
        sleep(1000);
        contractLift();
        sleep(750);
        stopLift();
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

    public void drive(int distance, int randomization) {
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
        
        telemetry.addData("motor1",motor1.getCurrentPosition());
        telemetry.update();

        // Sleep a bit to make sure the motor report as "busy"
        sleep(SLEEP_INTERVAL);
        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=motor1.getCurrentPosition();
            telemetry.addData("motor1", currentPosition);
            telemetry.addData("randomization",randomization);

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
            motor1.setPower(-1*direction*power);
            motor2.setPower(1*direction*power);
            motor3.setPower(-1*direction*power);
            motor4.setPower(1*direction*power);
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

    public void strafe(int distance, int randomization) {
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
        
        telemetry.addData("motor1",motor1.getCurrentPosition());
        telemetry.update();

        // Sleep a bit to make sure the motor report as "busy"
        sleep(SLEEP_INTERVAL);
        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=motor1.getCurrentPosition();
            telemetry.addData("motor1",currentPosition);
            telemetry.addData("randomization",randomization);
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("distance", getDistance());
            telemetry.addData("distanceL", getDistanceL());
            telemetry.addData("distanceR", getDistanceR());
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