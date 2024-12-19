package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    public GoBildaPinpointDriver odo;
    public PIDController pid;
    public MecanumMotionController mmc;
    public TeleMachine tm;
    
    //settings
    boolean specimen = false;
    boolean specimenPreload = false;
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
        //bot.setManualExposure(this, 2, 255);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(96.0, -75.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        
        tm = new TeleMachine(bot);
        
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);
        
        bot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        PID xPid = new PID(.12, .08, .02);//(.1,0.08,.01);
        PID yPid = new PID(.14, .08, .02);//(.12,0.08,.01);

        PID thetaPID = new PID(1.5,0.98,.09);//(2, 0.98, 0.08)
        thetaPID.errorSumTotal = .1;
        pid = new PIDController(bot, odo);
        
        pid.setPID(xPid, yPid);
        pid.setTurnPID(thetaPID);
        pid.maxAngSpeed = .4;
        pid.maxSpeed = .3;
        
        mmc = new MecanumMotionController(pid, bot, tm);
        
        
        
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
        
        // bot starts in tile F4 with a initial heading of 90 degrees
        tm.armClosed();
        bot.imu.resetYaw();
        if(specimen){
            Pose2D startPos = new Pose2D(DistanceUnit.INCH, -24, 64, AngleUnit.DEGREES, 0);
            odo.setPosition(startPos);
            mmc.setLastPos(-24, 64, 0);
        }else{
            Pose2D startPos = new Pose2D(DistanceUnit.INCH, 24, 64, AngleUnit.DEGREES, 0);
            odo.setPosition(startPos);
            mmc.setLastPos(24, 64, 0);
        }
        
        
    
        mmc.waitForSeconds(delay);
        
        //dropping preload
        if(specimenPreload){
            mmc.waitForSeconds(0, () -> tm.clawIsOpen = false);
            mmc.waitForSeconds(1, () -> tm.specimen());
            mmc.moveTo(0, 40, 90);
            mmc.waitForSeconds(0, () -> tm.specimenUp());
            mmc.moveTo(0, 45, 90, .5);
            mmc.waitForSeconds(.5, () -> tm.clawIsOpen = true);
        }else{
            mmc.waitForSeconds(0, () -> tm.clawIsOpen = false);
            mmc.waitForSeconds(1, () -> tm.goingToDrop());
            mmc.moveTo(55, 52, -45); //going to bucket for preload
            mmc.waitForSeconds(1);
            mmc.waitForSeconds(.5, () -> tm.clawIsOpen = true); //dropping preload
        }
        
        
        if(specimen){
            mmc.moveTo(0, 54, 90);
            mmc.waitForSeconds(0, () -> tm.clawIsOpen = true);
            mmc.waitForSeconds(0, () -> tm.goingToPickup());
            mmc.moveTo(-32, 60, 180);
            mmc.waitForSeconds(1, () -> tm.clawIsOpen = false);
            mmc.waitForSeconds(1, () -> tm.specimen());
            mmc.moveTo(-5, 48, 90);
            mmc.moveTo(-5, 40, 90);
            mmc.waitForSeconds(.5, () -> tm.specimenUp());
            mmc.moveTo(-5, 45, 90);
            mmc.waitForSeconds(0, () -> tm.clawIsOpen = true);
        }else{
            //first yellow
            mmc.moveTo(48, 48, -45); //backing up
            mmc.waitForSeconds(0, () -> tm.goingToPickup());
            mmc.moveTo(46.7, 48, 90); //lining up to grab sample
            mmc.waitForSeconds(.6);
            mmc.moveTo(46.7, 43.5, 90);
            grabAndDrop();
            
            //second yellow
            mmc.moveTo(48, 48, -45); //backing up
            mmc.waitForSeconds(0, () -> tm.goingToPickup());
            mmc.moveTo(56, 48, 90); //lining up to grab sample
            mmc.waitForSeconds(.6);
            mmc.moveTo(56, 43.5, 90);
            grabAndDrop();
            
            //third yellow
            mmc.moveTo(48, 48, -45); //backing up
            mmc.waitForSeconds(0, () -> tm.goingToPickup());
            mmc.moveTo(58, 46, 70); //lining up to grab sample
            mmc.waitForSeconds(.4);
            mmc.moveTo(60, 42, 72);
            mmc.waitForSeconds(.4);
            grabAndDrop();
            
            //parking
            mmc.moveTo(42, 48, 0);
            mmc.waitForSeconds(0, () -> tm.clawIsOpen = false);
            mmc.waitForSeconds(0, () -> tm.armClosing());
            mmc.moveTo(48, 24, 0);
            mmc.waitForSeconds(0, () -> bot.setFinger(.33));
            mmc.moveTo(20, 0, 0);
        }
        
        
        
        
        
            
        
        mmc.start();
        
//loop
    //repeating while active
        while (opModeIsActive()) {
            
            try{
                List<AprilTagDetection> currentDetections = bot.aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    //set odometry
                    double range = detection.ftcPose.range;
                    double bearing = detection.ftcPose.bearing;
                    double yaw = detection.ftcPose.yaw;
                    double tagx = detection.metadata.fieldPosition.get(0);
                    double tagy = detection.metadata.fieldPosition.get(1);
                    double theta = Math.toRadians(odo.getHeading() + bearing);
                    double fx = tagx - Math.cos(theta) * range;
                    fx -= 26;
                    double fy = tagy - Math.sin(theta) * range;
                    fy += 4;
                    //bot.resetEncoders();
                    //bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //od.setFieldXY(fx, fy);
                    telemetry.addLine("seeing april tag " + detection.id + " from " + fx + ", " + fy);
                }
            }
            catch( Exception e){
                
            }
            bot.updateEncoders();
            odo.update();
            mmc.update();
            tm.update();
            
            Pose2D currentPos = odo.getPosition();
            
            telemetry.addData("Status", "Running");
            telemetry.addData("position x", currentPos.getX(DistanceUnit.INCH));
            telemetry.addData("position y", currentPos.getY(DistanceUnit.INCH));
            telemetry.addData("target angle", pid.getTargetAngle());
            telemetry.addData("heading", bot.getHeading());
            telemetry.addData("FL", bot.getFLPos());
            telemetry.addData("FR", bot.getFRPos());
            telemetry.addData("BL", bot.getBLPos());
            telemetry.addData("BR", bot.getBRPos());
            telemetry.addLine();
            telemetry.addData(" sample x", bot.getSampleX(1));
            telemetry.update();
            
            
            
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
    
    public void grabAndDrop(){
        mmc.waitForSeconds(1);
        mmc.waitForSeconds(.5, () -> tm.clawIsOpen = false);
        mmc.waitForSeconds(.7, () -> tm.goingToDrop());
        mmc.moveTo(55.5, 52.5, -45); //going to bucket drop
        mmc.waitForSeconds(1.6);
        mmc.waitForSeconds(.5, () -> tm.clawIsOpen = true);
    }
}
