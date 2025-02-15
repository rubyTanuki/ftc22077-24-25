//extends Bot to hold an arm with a slide
//manages all drive train and arm
package Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static Main.BotValues.*;

public class TransferBot extends Bot{
    private DcMotorEx Outtake_Left;
    private DcMotorEx Outtake_Right;

    private Servo Intake_Left;
    private Servo Intake_Right;
    private Servo FlyWheel_Left;
    private Servo FlyWheel_Right;

    int outtakeLeftPos   = 0;
    int outtakeRightPos  = 0;
    
    int outtakeLeftPosPrev   = 0;
    int outtakeRightPosPrev  = 0;

    public TransferBot(HardwareMap hm)
    { //constructor method
        super(hm);
        initMotors(hm);
        initServos(hm);
        //resetEncoders();
    }

    private void initMotors(HardwareMap hm)
    { //initialising motors
        // motor1 = hm.get(DcMotorEx.class, "motor1");
        Outtake_Left = hm.get(DcMotorEx.class, "outtakeLeft");
        Outtake_Right = hm.get(DcMotorEx.class, "outtakeRight");
        
        // motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // motor1.setDirection(DcMotor.Direction.REVERSE); //REVERSE, FORWARD
        Outtake_Left.setDirection(DcMotor.Direction.FORWARD);
        Outtake_Right.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initServos(HardwareMap hm)
    { //initialising servos
        Intake_Left = hm.get(Servo.class, "intakeLeft");
        Intake_Right = hm.get(Servo.class, "intakeRight");
        FlyWheel_Left = hm.get(Servo.class, "flyWheelLeft");
        FlyWheel_Right = hm.get(Servo.class, "flyWheelRight");
    }
    
    public void setArmMode(DcMotor.RunMode mode)
    {
        Outtake_Left.setMode(mode);
        Outtake_Right.setMode(mode);
        //add other motors here
    }
    
    public void resetEncoders()
    { //Resetting encoders to 0, keeping current mode
        super.resetEncoders(); //Bot.resetEncoders()
        DcMotorEx.RunMode outLeftMode  = Outtake_Left.getMode();
        DcMotorEx.RunMode outRightMode = Outtake_Right.getMode();

        Outtake_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Outtake_Left.setMode(outLeftMode);
        Outtake_Right.setMode(outRightMode);
    }

    public void updateEncoders()
    { //updates current and last encoder positions
        super.updateEncoders(); //Bot.updateEncoders
        outtakeLeftPosPrev = outtakeLeftPos;
        outtakeRightPosPrev = outtakeRightPos;

        outtakeLeftPos =  Outtake_Left.getCurrentPosition();
        outtakeRightPos = Outtake_Right.getCurrentPosition();
    }
    
    //getter methods
    public DcMotorEx getOuttakeLeft()   { return Outtake_Left;   }
    public DcMotorEx getOuttakeRight()  { return Outtake_Right;  }
    public int getOuttakeLeftPos()      { return outtakeLeftPos; }
    public int getOuttakeRightPos()     { return outtakeRightPos;}

    public Servo getIntakeLeft()    { return Intake_Left;   }
    public Servo getIntakeRight()   { return Intake_Right;  }
    public Servo getFlyWheelLeft()  { return FlyWheel_Left; }
    public Servo getFlyWheelRight() { return FlyWheel_Right;}
    public double getIntakeLeftPos()    { return Intake_Left.getPosition();     }
    public double getIntakeRightPos()   { return Intake_Right.getPosition();    }
    public double getFlyWheelLeftPos()  { return FlyWheel_Left.getPosition();   }
    public double getFlyWheelRightPos() { return FlyWheel_Right.getPosition();  }

    // public void setArm(int armPos, int slidePos)
    // { //running arm and slide motors to requested position
    //     setArm(armPos);
    //     setSlide(slidePos);
    // }

    public void setOuttakeLeft(int pos)
    { //running left outtake motor to requested position
        Outtake_Left.setTargetPosition(pos);
        Outtake_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Outtake_Left.setVelocity(6000);
    }

    public void setOuttakeRight(int pos)
    { //running right outtake motor to requested position
        Outtake_Right.setTargetPosition(pos);
        Outtake_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Outtake_Right.setVelocity(6000);
    }
    

    public void setIntakeLeft(double pos)
    { //setting position of left intake servo
        Intake_Left.setPosition(pos);
    }
    public void setIntakeRight(double pos)
    { //setting position of left intake servo
        Intake_Right.setPosition(pos);
    }
    
    public void intake(){
        setIntakeLeft(1);
        setIntakeRight(0);
    }
    public void stopIntake(){
        setIntakeLeft(.5);
        setIntakeRight(.5);
    }
}
