package Main.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
public class BPad{
    Gamepad gpad;
    ElapsedTime timer = new ElapsedTime();

    private final double DBCLICKTIME = 1.0;

    boolean lastA;
    double lastATime;
    boolean lastB;
    double lastBTime;
    boolean lastX;
    double lastXTime;
    boolean lastY;
    double lastYTime;

    boolean lastUp;
    double lastUpTime;
    boolean lastDown;
    double lastDownTime;
    boolean lastRight;
    double lastRightTime;
    boolean lastLeft;
    double lastLeftTime;

    boolean lastStart;
    double lastStartTime;
    boolean lastGuide;
    double lastGuideTime;
    boolean lastBack;
    double lastBackTime;


    public BPad(Gamepad gpad){
        this.gpad = gpad;
        timer.reset();
    }

    //A, B, X, Y
    public boolean dbA(){lastATime = timer.seconds(); return !lastA && gpad.a; }
    public boolean dbB(){lastBTime = timer.seconds(); return !lastB && gpad.b; }
    public boolean dbX(){lastXTime = timer.seconds(); return !lastX && gpad.x; }
    public boolean dbY(){lastYTime = timer.seconds(); return !lastY && gpad.y; }

    public boolean doubleA(){ return dbA() && timer.seconds()-lastATime<DBCLICKTIME; }
    public boolean doubleB(){ return dbB() && timer.seconds()-lastBTime<DBCLICKTIME; }
    public boolean doubleX(){ return dbX() && timer.seconds()-lastXTime<DBCLICKTIME; }
    public boolean doubleY(){ return dbY() && timer.seconds()-lastYTime<DBCLICKTIME; }

    public boolean A(){ return gpad.a; }
    public boolean B(){ return gpad.b; }
    public boolean X(){ return gpad.x; }
    public boolean Y(){ return gpad.y; }

    //Up, Down, Left, Right
    public boolean dbUp(){lastUpTime = timer.seconds(); return !lastUp && gpad.dpad_up;}
    public boolean dbDown(){lastDownTime = timer.seconds(); return !lastDown && gpad.dpad_down;}
    public boolean dbLeft(){lastLeftTime = timer.seconds(); return !lastLeft && gpad.dpad_left;}
    public boolean dbRight(){lastRightTime = timer.seconds(); return !lastRight && gpad.dpad_right;}

    public boolean doubleUp(){ return dbUp() && timer.seconds()-lastUpTime<DBCLICKTIME; }
    public boolean doubleDown(){ return dbDown() && timer.seconds()-lastDownTime<DBCLICKTIME; }
    public boolean doubleLeft(){ return dbLeft() && timer.seconds()-lastLeftTime<DBCLICKTIME; }
    public boolean doubleRight(){ return dbRight() && timer.seconds()-lastRightTime<DBCLICKTIME; }

    public boolean Up(){ return gpad.dpad_up;}
    public boolean Down(){ return gpad.dpad_down;}
    public boolean Left(){ return gpad.dpad_left;}
    public boolean Right(){ return gpad.dpad_right;}

    //Bumpers and Triggers
    public boolean leftBumper(){ return gpad.left_bumper; }
    public double leftTrigger(){ return gpad.left_trigger; }
    public boolean rightBumper(){ return gpad.right_bumper; }
    public double rightTrigger(){ return gpad.right_trigger; }

    //Sticks
    public double leftStickX(){ return gpad.left_stick_x; }
    public double leftStickY(){ return gpad.left_stick_y; }
    public double rightStickX(){ return gpad.right_stick_x; }
    public double rightStickY(){ return gpad.right_stick_y; }

    //Funny buttons
    public boolean dbStart(){ return !lastStart && start(); }
    public boolean dbGuide(){ return !lastGuide && guide(); }
    public boolean dbBack(){ return !lastBack && back(); }

    public boolean doubleStart(){ return dbStart() && timer.seconds()-lastStartTime<DBCLICKTIME; }
    public boolean doubleGuide(){ return dbGuide() && timer.seconds()-lastGuideTime<DBCLICKTIME; }
    public boolean doubleBack(){ return dbBack() && timer.seconds()-lastBackTime<DBCLICKTIME; }

    public boolean start(){ return gpad.start; }
    public boolean guide(){ return gpad.guide; }
    public boolean back(){ return gpad.back; }
    


    public void update(){
        lastA = gpad.a;
        lastB = gpad.b;
        lastX = gpad.x;
        lastY = gpad.y;
        lastUp = gpad.dpad_up;
        lastDown = gpad.dpad_down;
        lastLeft = gpad.dpad_left;
        lastRight = gpad.dpad_right;
        lastStart = gpad.start;
        lastGuide = gpad.guide;
        lastBack = gpad.back;
    }
}