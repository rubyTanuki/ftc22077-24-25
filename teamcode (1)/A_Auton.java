package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class A_Auton extends LinearOpMode {

    public BaseBot bot;
    
    @Override
    public void runOpMode() {
        
        bot = new BaseBot();
        bot.init(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // bot starts in tile F4 with a initial heading of 90 degrees
        bot.setHeading(90);
        bot.setFieldXY(0, 0);
        
        BaseBot.Pose[] path = new BaseBot.Pose[]{
            new BaseBot.Pose(-20, 20, 90),
            new BaseBot.Pose(-20, 50, 90),
            new BaseBot.Pose(20, 20, 90),
        };
        BaseBot.Pose[] path2 = new BaseBot.Pose[]{
            new BaseBot.Pose(0, 0, 90)
        };
        
        
        followPath(path);
        followPath(path2);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

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
}
