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

@Autonomous(name = "A_Auton")

public class A_Auton extends LinearOpMode {

    public BaseBot bot;
    
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
        bot = new BaseBot();
        bot.init(hardwareMap);
        bot.setManualExposure(this, 2, 255);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
//init_loop
    //choosing settings with controller
        
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
        waitForStart();
//start
    //adding all the movement paths
    BaseBot.Pose[] preloadPath = new BaseBot.Pose[1];
    // bot starts in tile F4 with a initial heading of 90 degrees
    bot.setHeading(0);
    bot.setFieldXY(0, 0);
    
    //specimen
        if(specimen){
            //use preload
            if(specimenPreload){
                preloadPath = new BaseBot.Pose[]{
                    new BaseBot.Pose(0, 0, 0)
                };
            }else{
                
            }
            //cycle
            
            
            
    //bucket
        }else{
            //use preload
            
            //cycle
            
            
        }
    
    
    
        
       
        
        
        
        waitForSeconds(2);
        followPath(preloadPath);
        waitForSeconds(5);
        
        
//loop
    //repeating while active
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            List<AprilTagDetection> currentDetections = bot.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("tag", bot.format(detection));
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
    
    
    public void followPath(BaseBot.Pose[] path){
        int i = 0;
        while (i<path.length){
            if(!opModeIsActive()) return;
            bot.updateTracking();
            double dist = bot.driveToPose(path[i], .7);
            telemetry.addData("path target", i);
            telemetry.addData("target pose", path[i]);
            telemetry.addData("field pose", bot.field);
            telemetry.addData("dist", dist);
            telemetry.update();
            if(dist < 1.0) i++;
        }
        bot.driveXYW(0, 0, 0);
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
