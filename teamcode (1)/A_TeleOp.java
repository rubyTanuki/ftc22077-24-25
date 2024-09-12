package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;


@TeleOp(name="TeleOpOmni", group="Linear OpMode")

public class A_TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    
    
    public BaseBot bot;
    
    private boolean lastA = false;
    private boolean grip = false;
    private int targetId = 16;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        bot = new BaseBot();
        bot.init(hardwareMap);
        bot.setManualExposure(this, 2, 255);
        
        waitForStart();
        runtime.reset();
        
        BaseBot.Pose targetPose = new BaseBot.Pose(24, 0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            bot.updateTracking();
            telemetry.addData("field pose", bot.field);
            telemetry.addData("targetPose", targetPose);
            
            double multi = gamepad1.left_trigger;
            double jx = -gamepad1.left_stick_y;
            double jy = -gamepad1.left_stick_x;
            double jw = -gamepad1.right_stick_x;

            if (gamepad1.start) {
                if (gamepad1.dpad_up) bot.setHeading(0);
            }

            driveFieldXYW(jx*Math.max(.25, (1-multi)), jy*Math.max(.25, (1-multi)), jw*Math.max(.25, (1-multi)));
            
            if(gamepad1.a && !lastA){
                grip = !grip;
            }
            
            if(grip){
                bot.setClaw(0);
            }else{
                bot.setClaw(.35);
            }
            
            //april tag processing
            List<AprilTagDetection> currentDetections =
                    bot.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if(detection.id == targetId){
                    //process tag
                }
                telemetry.addData("tag", bot.format(detection));
                
            }
            
            lastA = gamepad1.a;
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading", bot.getHeading());
            telemetry.addData("bot x", bot.)
            telemetry.update();
        }
    }
    
    public void driveFieldXYW(double fx, double fy, double fw) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(bot.getHeading());
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        bot.driveXYW(rx, ry, fw);
    }
}
