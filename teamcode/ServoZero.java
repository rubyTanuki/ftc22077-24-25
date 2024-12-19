package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="ServoZero", group="Linear OpMode")

public class ServoZero extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo pitch;
    private Servo roll;
    private Servo claw;
    private Servo finger;
    
    private double pitchPos = .5;
    private double rollPos = .5;
    private double clawPos = .5;
    private double fingerPos = .5;
    
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pitch = hardwareMap.get(Servo.class, "pitch");
        roll = hardwareMap.get(Servo.class, "roll");
        claw = hardwareMap.get(Servo.class, "claw");
        finger = hardwareMap.get(Servo.class, "finger");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean x = gamepad1.x;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean y = gamepad1.y;
            
            
            if(a && !lastA) pitchPos += gamepad1.back?0.05:0.1;
            if(pitchPos > 1) pitchPos = 0;
            
            if(b && !lastB) rollPos += gamepad1.back?0.05:0.1;
            if(rollPos>1) rollPos = 0;
            
            if(x && !lastX) clawPos += gamepad1.back?0.05:0.1;
            if(clawPos>1) clawPos = 0;
            
            if(y && !lastY) fingerPos += gamepad1.back?0.05:0.1;
            if(fingerPos>1) fingerPos = 0;
            
            
            
            
            if(gamepad1.left_trigger>0){
                pitch.setPosition(pitchPos);
                roll.setPosition(rollPos);
                claw.setPosition(clawPos);
                finger.setPosition(fingerPos);
            }
            if(gamepad1.dpad_left){
                claw.setPosition(clawPos);
            }
            if(gamepad1.dpad_right){
                roll.setPosition(rollPos);
            }
            if(gamepad1.dpad_up){
                finger.setPosition(fingerPos);
            }
            if(gamepad1.dpad_down){
                pitch.setPosition(pitchPos);
            }
            
            
            lastA = a;
            lastB = b;
            lastX = x;
            lastY = y;
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Claw", clawPos);
            telemetry.addData("Pitch", pitchPos);
            telemetry.addData("Roll", rollPos);
            telemetry.addData("Finger", fingerPos);
            telemetry.update();
        }
    }
}
