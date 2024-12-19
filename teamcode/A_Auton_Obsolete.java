// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import java.util.List;
// import Main.*;
// import Main.auton.*;
// 
// @Autonomous(name = "A_Auton")
// 
// public class A_Auton_Obsolete extends LinearOpMode {
// 
//     public AutoBot bot;
//     public BotOdometry od;
//     public PIDController pid;
//     public MecanumMotionController mmc;
//     public TeleMachine tm;
//     
//     //settings
//     boolean specimen = false;
//     boolean specimenPreload = false;
//     int cycles = 0;
//     int delay = 0;
//     
//     
//     //debouncing
//     boolean lastA = false;
//     boolean lastB = false;
//     boolean lastX = false;
//     boolean lastY = false;
//     boolean lastStart = false;
//     boolean lastBack = false;
//     
//     
//     
//     
//     @Override
//     public void runOpMode() {
// //init
//     //hardwaremaps and init methods
//         bot = new AutoBot(hardwareMap);
//         //bot.setManualExposure(this, 2, 255);
//         od = new BotOdometry(bot);
//         
//         tm = new TeleMachine(bot);
//         
//         bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         bot.enableBrakeMode(true);
//         
//         bot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         
//         
//         PID xPid = new PID(.09, .08, .01);//(.1,0.08,.01);
//         PID yPid = new PID(.12, .08, .008);//(.12,0.08,.01);
// 
//         PID thetaPID = new PID(0.6,0.98,.08);//(2, 0.98, 0.08)
//         thetaPID.errorSumTotal = .1;
//         pid = new PIDController(bot, od);
//         
//         pid.setPID(xPid, yPid);
//         pid.setTurnPID(thetaPID);
//         pid.maxAngSpeed = .4;
//         pid.maxSpeed = .3;
//         
//         mmc = new MecanumMotionController(pid, bot, tm);
//         
//         
//         
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
//         
// //init_loop
//     //choosing settings with controller
//         /*
//         int minSelector = 1;
//         int maxSelector = 4;
//         int selector = minSelector;
//         while(opModeInInit()){
//             
//             //changing selector
//             if(gamepad1.a && selector!=maxSelector && !lastA) selector++;
//             if(gamepad1.y  && selector!=minSelector && !lastY) selector--;
//             
//             //changing setting values
//             int change = 0;
//             if(gamepad1.b && !lastB) change = 1;
//             if(gamepad1.x && !lastX) change = -1;
//             if(change!=0){
//                 switch(selector){
//                     case 1:
//                         specimen = !specimen;
//                         break;
//                     case 2:
//                         specimenPreload = !specimenPreload;
//                         break;
//                     case 3:
//                         cycles = cycles + change;
//                         break;
//                     case 4:
//                         delay = delay + change;
//                         break;
//                 }
//             }
//             lastButtons(); //updating debouncing variables
//             
//             //updating telemetry
//             telemetry.addLine((selector==1?">":" ") + "SPECIMEN = " + specimen);
//             telemetry.addLine((selector==2?">":" ") + "SPECIMEN PRELOAD = " + specimenPreload);
//             telemetry.addLine((selector==3?">":" ") + "CYCLE COUNT = " + cycles);
//             telemetry.addLine((selector==4?">":" ") + "DELAY SECONDS = " + delay);
//             telemetry.update();
//             
//         }
//         */
//         
//         
//         waitForStart();
// //start
//         //adding all the movement paths
//         
//         // bot starts in tile F4 with a initial heading of 90 degrees
//         tm.armClosed();
//         bot.imu.resetYaw();
//         od.setFieldXY(24, 64);
//         mmc.setLastPos(24, 64, 0);
//     
//     
//         
//         
//             
//             
//             
//             
//         //dropping preload
//         mmc.waitForSeconds(0, () -> tm.clawIsOpen = false);
//         mmc.waitForSeconds(1.5, () -> tm.goingToDrop());
//         mmc.moveTo(65, 52.5, -45);
//         mmc.waitForSeconds(1);
//         mmc.waitForSeconds(.5, () -> tm.clawIsOpen = true);
//         
//         //first yellow
//         mmc.moveTo(48, 48, -45); //backing up
//         mmc.waitForSeconds(0, () -> tm.goingToPickup());
//         mmc.moveTo(35, 48, 0);
//         mmc.waitForSeconds(1);
//         mmc.moveTo(50, 54, 90); //lining up to grab sample
//         mmc.waitForSeconds(.5);
//         mmc.alignToSample(1, 5);
//         mmc.waitForSeconds(0, () -> od.setFieldXY(47, 56));
//         mmc.waitForSeconds(0, () -> mmc.setLastPos(47, 56, 90));
//         mmc.waitForSeconds(.5);
//         mmc.moveTo(999, 44, 999); //grabbing sample
//         mmc.waitForSeconds(.8);
//         mmc.waitForSeconds(1, () -> tm.clawIsOpen = false);
//         mmc.waitForSeconds(2, () -> tm.goingToDrop());
//         mmc.moveTo(57, 58, -45); //going to bucket drop
//         mmc.waitForSeconds(1);
//         mmc.waitForSeconds(.5, () -> tm.clawIsOpen = true);
//         mmc.moveTo(40, 48, 0);
//         
//         //second yellow
//         mmc.waitForSeconds(1, () -> tm.goingToPickup());
//         mmc.moveTo(56, 54, 90); //lining up to grab sample
//         mmc.waitForSeconds(.5);
//         mmc.alignToSample(1, 5);
//         mmc.waitForSeconds(0, () -> od.setFieldXY(57, 56));
//         mmc.waitForSeconds(0, () -> mmc.setLastPos(57, 56, 90));
//         mmc.waitForSeconds(.4);
//         mmc.moveTo(999, 43, 999); //grabbing sample
//         mmc.waitForSeconds(.5);
//         mmc.waitForSeconds(.5, () -> tm.clawIsOpen = false);
//         mmc.waitForSeconds(1.5, () -> tm.goingToDrop());
//         mmc.moveTo(48, 48, 0);
//         mmc.moveTo(57, 58, -45); //going to bucket drop
//         mmc.waitForSeconds(1);
//         mmc.waitForSeconds(.5, () -> tm.clawIsOpen = true);
//         
//         //parking
//         mmc.moveTo(42, 48, 0);
//         mmc.waitForSeconds(0, () -> tm.clawIsOpen = false);
//         mmc.waitForSeconds(0, () -> tm.armClosing());
//         mmc.moveTo(28, 0, 90);
//             
//         
//         mmc.start();
//         
// //loop
//     //repeating while active
//         while (opModeIsActive()) {
//             
//             try{
//                 List<AprilTagDetection> currentDetections = bot.aprilTag.getDetections();
//                 for (AprilTagDetection detection : currentDetections) {
//                     //set odometry
//                     double range = detection.ftcPose.range;
//                     double bearing = detection.ftcPose.bearing;
//                     double yaw = detection.ftcPose.yaw;
//                     double tagx = detection.metadata.fieldPosition.get(0);
//                     double tagy = detection.metadata.fieldPosition.get(1);
//                     double theta = Math.toRadians(od.getHeading() + bearing);
//                     double fx = tagx - Math.cos(theta) * range;
//                     fx -= 26;
//                     double fy = tagy - Math.sin(theta) * range;
//                     fy += 4;
//                     //bot.resetEncoders();
//                     //bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                     //od.setFieldXY(fx, fy);
//                     telemetry.addLine("seeing april tag " + detection.id + " from " + fx + ", " + fy);
//                 }
//             }
//             catch( Exception e){
//                 
//             }
//             bot.updateEncoders();
//             od.updateTracking();
//             mmc.update();
//             tm.update();
//             
//             
//             telemetry.addData("Status", "Running");
//             telemetry.addData("position x", od.getX());
//             telemetry.addData("position y", od.getY());
//             telemetry.addData("target angle", pid.getTargetAngle());
//             telemetry.addData("heading", bot.getHeading());
//             telemetry.addData("FL", bot.getFLPos());
//             telemetry.addData("FR", bot.getFRPos());
//             telemetry.addData("BL", bot.getBLPos());
//             telemetry.addData("BR", bot.getBRPos());
//             telemetry.addLine();
//             telemetry.addData(" sample x", bot.getSampleX(1));
//             telemetry.update();
//             
//             
//             
//         }
//     }
//     
//     
//     
//     
//     
//     public void waitForSeconds(double seconds){
//         ElapsedTime timer = new ElapsedTime();
//         while(true){
//             if(timer.seconds()>seconds) break;
//             telemetry.addLine("Waiting...");
//         }
//     }
//     public void waitForSeconds(double seconds, Runnable command){
//         ElapsedTime timer = new ElapsedTime();
//         command.run();
//         while(true){
//             if(timer.seconds()>seconds) break;
//             telemetry.addLine("Waiting...");
//         }
//     }
//     
//     
//     public void lastButtons(){
//         lastA = gamepad1.a;
//         lastB = gamepad1.b;
//         lastX = gamepad1.x;
//         lastY = gamepad1.y;
//         lastStart = gamepad1.start;
//         lastBack = gamepad1.back;
//     }
// }
// 
