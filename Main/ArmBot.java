//extends Bot to hold an slide_right with a slide
//manages all drive train and slide_right
package Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static Main.BotValues.*;

public class ArmBot extends Bot{
    private DcMotorEx slide_right;
    private DcMotorEx slide_left;
    public DcMotorEx intake;

    private Servo intake_pitch;
    private Servo claw_pitch;
    private Servo intake_out;
    private Servo forearm;
    private Servo claw;

    int slide_rightPos   = 0;
    int slide_leftPos = 0;
    int intakePos = 0;

    int slide_rightPosPrev = 0;
    int slide_leftPosPrev = 0;

    public ArmBot(HardwareMap hm)
    { //constructor method
        super(hm);
        initMotors(hm);
        initServos(hm);
        //resetEncoders();
    }

    private void initMotors(HardwareMap hm)
    { //initialising motors
        slide_right = hm.get(DcMotorEx.class, "slide_right");
        slide_left = hm.get(DcMotorEx.class, "slide_left");
        intake = hm.get(DcMotorEx.class, "intake");
        
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        slide_right.setDirection(DcMotor.Direction.FORWARD);
        slide_left.setDirection(DcMotor.Direction.REVERSE);
        
        resetEncoders();
    }

    private void initServos(HardwareMap hm)
    { //initialising servos
        claw_pitch = hm.get(Servo.class, "clawPitch");
        intake_pitch = hm.get(Servo.class, "intakePitch");
        intake_out = hm.get(Servo.class, "intakeOut");
        forearm = hm.get(Servo.class, "forearm");
        claw = hm.get(Servo.class, "claw");
        
    }
    
    public void setSlideMode(DcMotor.RunMode mode)
    {
        slide_right.setMode(mode);
        slide_left.setMode(mode);
    }
    
    public void resetEncoders()
    { //Resetting encoders to 0, keeping current mode
        super.resetEncoders(); //Bot.resetEncoders()
        DcMotorEx.RunMode slide_rightMode = slide_right.getMode();
        DcMotorEx.RunMode slide_leftMode = slide_left.getMode();

        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide_right.setMode(slide_rightMode);
        slide_left.setMode(slide_leftMode);
    }

    public void updateEncoders()
    { //updates current and last encoder positions
        super.updateEncoders(); //Bot.updateEncoders
        slide_rightPosPrev = slide_rightPos;
        slide_leftPosPrev = slide_leftPos;

        slide_rightPos = slide_right.getCurrentPosition();
        slide_leftPos = slide_left.getCurrentPosition();
        intakePos = intake.getCurrentPosition();
    }
    
    //getter methods
    public DcMotorEx getSlideRight()   { return slide_right;   }
    public DcMotorEx getSlideLeft() { return slide_left; }
    public DcMotorEx getIntake()  { return intake;  }
    public int getArmPos()      { return slide_rightPos;   }
    public int getSlidePos()    { return slide_leftPos; }
    public int getIntakePos()     { return intakePos;  }

    public Servo getIntakePitch()   { return intake_pitch;  }
    public Servo getClawPitch()     { return claw_pitch;    }
    public Servo getForearm()   { return forearm;   }
    public Servo getClaw()      { return claw;      }
    public double getIntakePitchPos()     { return intake_pitch.getPosition();   }
    public double getClawPitchPos()     { return claw_pitch.getPosition();   }
    public double getForearmPos()   { return forearm.getPosition(); }
    public double getClawPos()      { return claw.getPosition();    }

    
    int slideTarget = 0;
    public void setSlidesTarget(int pos){
        slideTarget = pos;
    }
    public void setSlides(int pos){
        setSlidesTarget(pos);
        setSlides();
    }
    public void setSlides(){
        boolean stuck_right = Math.abs(slide_rightPos - slideTarget)<100 && 
                        slide_right.getVelocity()<10;
        
        if(Math.abs(slide_rightPos-slideTarget)>25 && !stuck_right){
            slide_right.setTargetPosition(slideTarget);
            slide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide_right.setVelocity(12000);
        }else{
            slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            slide_right.setPower(0);
        }
        
        // boolean stuck_left = Math.abs(slide_leftPos - slideTarget)<100 && 
        //                 slide_left.getVelocity()<10;
        
        // if(Math.abs(slide_leftPos-slideTarget)>25 && !stuck_left){
        //     slide_left.setTargetPosition(slideTarget);
        //     slide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     slide_left.setVelocity(12000);
        // }else{
        //     slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        //     slide_left.setPower(0);
        // }
    }
    
    public void setOuttake(int slidePos, double forearmPos, double pitchPos){
        setSlides(slidePos);
        forearm.setPosition(forearmPos);
        claw_pitch.setPosition(pitchPos);
    }
    
    public void setForearm(double pos){
        forearm.setPosition(pos);
    }
    public void setClawPitch(double pos){
        claw_pitch.setPosition(pos);
    }
    
}
