package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import Main.*;
import Main.auton.*;

@Autonomous(name = "A_Auton")

public class A_Auton extends LinearOpMode {

    public AutoBot bot;
    public BotOdometry od;
    public PIDController pid;
    public MecanumMotionController mmc;
    
    //settings
    boolean specimen = true;
    boolean specimenPreload = true;
    int cycles = 0;
    int delay = 0;
    
    
    //debouncing
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastY = false;
    boolean lastStart = false;
    boolean lastBack = false;
    
    
    
    
    @Override
    public void runOpMode() {
//init
    //hardwaremaps and init methods
        bot = new AutoBot(hardwareMap);
        bot.setManualExposure(this, 2, 255);
        od = new BotOdometry(bot);
        
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);
        
        bot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        PID xPid = new PID(.07, .07, .01);//(.1,0.08,.01);
        PID yPid = new PID(.12, .08, .01);//(.12,0.08,.01);

        PID thetaPID = new PID(1,0.98,.08);//(2, 0.98, 0.08)
        thetaPID.errorSumTotal = .1;
        pid = new PIDController(bot, od);
        
        pid.setPID(xPid, yPid);
        pid.setTurnPID(thetaPID);
        pid.maxAngSpeed = .9;
        pid.maxSpeed = 1;
        
        mmc = new MecanumMotionController(pid, bot);
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
//init_loop
    //choosing settings with controller
        /*
        int minSelector = 1;
        int maxSelector = 4;
        int selector = minSelector;
        while(opModeInInit()){
            
            //changing selector
            if(gamepad1.a && selector!=maxSelector && !lastA) selector++;
            if(gamepad1.y  && selector!=minSelector && !lastY) selector--;
            
            //changing setting values
            int change = 0;
            if(gamepad1.b && !lastB) change = 1;
            if(gamepad1.x && !lastX) change = -1;
            if(change!=0){
                switch(selector){
                    case 1:
                        specimen = !specimen;
                        break;
                    case 2:
                        specimenPreload = !specimenPreload;
                        break;
                    case 3:
                        cycles = cycles + change;
                        break;
                    case 4:
                        delay = delay + change;
                        break;
                }
            }
            lastButtons(); //updating debouncing variables
            
            //updating telemetry
            telemetry.addLine((selector==1?">":" ") + "SPECIMEN = " + specimen);
            telemetry.addLine((selector==2?">":" ") + "SPECIMEN PRELOAD = " + specimenPreload);
            telemetry.addLine((selector==3?">":" ") + "CYCLE COUNT = " + cycles);
            telemetry.addLine((selector==4?">":" ") + "DELAY SECONDS = " + delay);
            telemetry.update();
            
        }
        */
        waitForStart();
//start
    //adding all the movement paths
    
    // bot starts in tile F4 with a initial heading of 90 degrees
    bot.imu.resetYaw();
    od.setFieldXY(-12, -65);
    mmc.setLastPos(-12, -65, 0);
    
    //mmc.moveTo(30, 45, 90);
    //mmc.waitForSeconds(3, () -> bot.setArm(1100, 0));
    //mmc.waitForSeconds(3, () -> bot.setArm(1100, 500));
    
    mmc.waitForSeconds(0, () -> bot.setArm(2000, 0));
    mmc.moveTo(-5, -55.5, -90);
    //set wrist
    mmc.moveArmTo(2000, 200);
    //release claw
    mmc.moveTo(-5, -59, -90, () -> bot.setArm(1900, 20));
    //adjust wrist
    mmc.moveArmTo(0, 100);
    mmc.moveTo(-31, -59, -90); //going to pick up first sample
    mmc.waitForSeconds(1);
    mmc.moveTo(-31, -55, -90); //going forward to grab
    mmc.waitForSeconds(1);
    //grab
    mmc.moveArmTo(2750, 1700); //moving arm up for outtake
    mmc.moveTo(-31.5, -62, 135); //moving to outtake
    mmc.waitForSeconds(1);
    mmc.moveArmTo(3000, 50); //moving arm back to take second specimen
    mmc.waitForSeconds(0, () -> mmc.moveArmTo(0, 100));
    mmc.moveTo(-35, -59, -90);
    
    
    
    
    
    // //specimen
    //     if(specimen){
    //         //use preload
    //         if(specimenPreload){
    //             mmc.moveTo(0, 35, 90);
    //             mmc.waitForSeconds(2);
    //             //put on bar
    //         }else{
                
    //         }
    //         //cycle
    //         mmc.moveTo(60, 45, 90);
    //         mmc.waitForSeconds(2);
    //         mmc.moveTo(60, 55, -45);
    //         mmc.waitForSeconds(2);
            
            
            
    // //bucket
    //     }else{
    //         //use preload
            
    //         //cycle
            
            
    //     }
    
    
    
        
       
        
        
        
        
        mmc.start();
        
//loop
    //repeating while active
        while (opModeIsActive()) {
            bot.updateEncoders();
            od.updateTracking();
            //pid.update();
            mmc.update();
            
            
            telemetry.addData("Status", "Running");
            telemetry.addData("position x", od.getX());
            telemetry.addData("position y", od.getY());
            telemetry.addData("target x", pid.targetX);
            telemetry.addData("target y", pid.targetY);
            telemetry.addData("target angle", pid.getTargetAngle());
            telemetry.addData("heading", bot.getHeading());
            telemetry.addData("odo heading", od.getHeading());
            telemetry.addData("FL", bot.getFLPos());
            telemetry.addData("FR", bot.getFRPos());
            telemetry.addData("BL", bot.getBLPos());
            telemetry.addData("BR", bot.getBRPos());
            telemetry.update();
            
            try{
                List<AprilTagDetection> currentDetections = bot.aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    //set odomentry
                    double range = detection.ftcPose.range;
                    double bearing = detection.ftcPose.bearing;
                    double yaw = detection.ftcPose.yaw;
                    double tagx = detection.metadata.fieldPosition.get(0);
                    double tagy = detection.metadata.fieldPosition.get(1);
                    double theta = Math.toRadians(od.getHeading() + bearing);
                    double fx = tagx - Math.cos(theta) * range;
                    double fy = tagy - Math.sin(theta) * range;
                    od.setFieldXY(fx, fy);
                }
            }
            catch( Exception e){
                
            }
            
        }
    }
    
    
    
    
    
    public void waitForSeconds(double seconds){
        ElapsedTime timer = new ElapsedTime();
        while(true){
            if(timer.seconds()>seconds) break;
            telemetry.addLine("Waiting...");
        }
    }
    public void waitForSeconds(double seconds, Runnable command){
        ElapsedTime timer = new ElapsedTime();
        command.run();
        while(true){
            if(timer.seconds()>seconds) break;
            telemetry.addLine("Waiting...");
        }
    }
    
    
    public void lastButtons(){
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastStart = gamepad1.start;
        lastBack = gamepad1.back;
    }
}
