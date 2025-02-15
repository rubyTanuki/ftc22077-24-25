package Main.shorty;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
import Main.util.*;
import Main.auton.*;


@TeleOp(name="ATELEOP", group="Linear OpMode")

public class Teleop_Shorty extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    
    
    public ArmBot bot;
    public Arm_FSM armSM;
    public GoBildaPinpointDriver odo;
    // public PIDController pid;
    // public MecanumMotionController mmc;


    public BPad gpad1;
    public BPad gpad2;
    
    

    public boolean subroutineIsActive;
    public boolean specimen;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        initialization();
        
        
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        waitForStart();
        runtime.reset();
        

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            gpad1.update();
            gpad2.update();
            
            if(subroutineIsActive){
                //mmc.update();
            }else if(false){ //true to activate drive train
                double multi = gpad1.leftTrigger();
                double jx = -gpad1.leftStickY();
                double jy = -gpad1.leftStickX();
                double jw = -gpad1.rightStickX();

                if(gpad1.dbStart()) { bot.resetHeading(); }
                if(gpad1.dbUp()){ bot.imu.resetYaw(); }

                double clampedMulti = Math.max(.25, (1-multi));
                driveFieldXYW(jx*clampedMulti, jy*clampedMulti, jw*clampedMulti);
                
                
            }

            if(gamepad1.timestamp<1 || gamepad2.timestamp<1) subroutineIsActive = false;
            
            if(gamepad2.a){
                armSM.setState("OUTTAKE_UP");
            }
            if(gamepad2.b){
                armSM.setState("OUTTAKE_DOWN");
            }




            bot.updateEncoders();
            odo.update();
            armSM.update();
            
            Pose2D pos = odo.getPosition();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("X ", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y ", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addData("subroutine is active", subroutineIsActive);
            telemetry.addData("currentState", armSM.toString());
            telemetry.update();
        }
    }
    
    public void driveFieldXYW(double fx, double fy, double fw) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(-bot.getHeading());
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        bot.driveXYW(rx, ry, fw);
    }


    public void initialization(){
        if(gamepad1!=null) gpad1 = new BPad(gamepad1);
        if(gamepad2!=null) gpad2 = new BPad(gamepad2);
        
        
        bot = new ArmBot(hardwareMap);
        //bot.setManualExposure(this, 2, 255);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(96.0, -75.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        
        armSM = new Arm_FSM(bot, gpad1, gpad2);
        
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.enableBrakeMode(true);
        
        //bot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        // PID xPid = new PID(.12, .08, .02);//(.1,0.08,.01);
        // PID yPid = new PID(.14, .08, .02);//(.12,0.08,.01);

        // PID thetaPID = new PID(1.5,0.98,.09);//(2, 0.98, 0.08)
        // thetaPID.errorSumTotal = .1;
        // pid = new PIDController(bot, odo);
        
        // pid.setPID(xPid, yPid);
        // pid.setTurnPID(thetaPID);
        // pid.maxAngSpeed = .4;
        // pid.maxSpeed = .3;
        
        //mmc = new MecanumMotionController(pid, bot, tm);
        
        // Pose2D startPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        // odo.setPosition(startPos);
        //mmc.setLastPos(0, 0, 0);
    }

    // public void moveToZero(){
    //     mmc.clearList();
    //     mmc.resetCurrent();
    //     mmc.moveTo(0,0,0);
    //     mmc.waitForSeconds(0, ()->subroutineIsActive=false);
    //     subroutineIsActive = true;
    // }
}