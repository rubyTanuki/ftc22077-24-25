package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import Main.*;


@TeleOp(name="TeleOpOmni", group="Linear OpMode")

public class A_TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    
    
    public AutoBot bot;
    public TeleMachine tm;

    public boolean lastX2;
    public boolean lastY2;
    public boolean lastA2;
    public boolean lastB2;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        
        bot = new AutoBot(hardwareMap);
        tm = new TeleMachine(bot);
        
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;
            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            
            double multi = gamepad1.left_trigger;
            double jx = -gamepad1.left_stick_y;
            double jy = -gamepad1.left_stick_x;
            double jw = -gamepad1.right_stick_x;

            if (gamepad1.start) {
                if (gamepad1.dpad_up) bot.resetHeading();
            }
            
            if(gamepad1.dpad_up){ bot.imu.resetYaw(); }

            driveFieldXYW(jx*Math.max(.25, (1-multi)), jy*Math.max(.25, (1-multi)), jw*Math.max(.25, (1-multi)));
            
            if(x2 && !lastX2){ tm.armClosing();     tm.start();}
            if(y2 && !lastY2){ tm.goingToPickup();  tm.start();}
            if(b2 && !lastB2){ tm.goingToDrop();    tm.start();}
            if(a2 && !lastA2){ tm.specimen();       tm.start();}
            
            bot.updateEncoders();
            tm.update();
            tm.updateGamepad(gamepad1, gamepad2);
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading", bot.getHeading());
            telemetry.addData("currentState", tm.toString());
            telemetry.addData("arm position", bot.getArmPos());
            telemetry.addData("slide position", bot.getSlidePos());
            telemetry.update();

            lastX2 = x2;
            lastY2 = y2;
            lastA2 = a2;
            lastB2 = b2;
        }
    }
    
    public void driveFieldXYW(double fx, double fy, double fw) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(-bot.getHeading());
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        bot.driveXYW(rx, ry, fw);
    }
}
