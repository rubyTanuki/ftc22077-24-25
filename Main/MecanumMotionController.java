// package Main;
// 
// import com.qualcomm.robotcore.util.ElapsedTime;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// import org.firstinspires.ftc.robotcore.external.State;
// import Main.util.Mathf;
// import Main.*;
// import Main.auton.*;
// import org.firstinspires.ftc.teamcode.*;
// 
// import java.util.ArrayList;
//     
// public class MecanumMotionController {
// 
//     
//     PIDController pid = null;
//     AutoBot bot = null;
//     TeleMachine tm = null;
//     ArrayList<MecanumAction> actions = new ArrayList<MecanumAction>();
//     int current = 0;
//     
//     public MecanumMotionController(PIDController pid, AutoBot bot, TeleMachine tm){
//         this.pid = pid;
//         this.bot = bot;
//         this.tm = tm;
//     }
//     
//     public void start()
//     {
//         actions.get(current).start();
//     }
//     
//     public void update(){
//         if(true){//ended == false){
//             actions.get(current).update();
//         }
//     }
//     
//     public void end(){
//         
//     }
//     public double lastX = 0;
//     public double lastY = 0;
//     public double lastAngle = 0;
//     
//     public void setLastPos(double x, double y, double theta){
//         lastX = x; lastY = y; lastAngle = theta;
//     }
//     
//     
//     public void alignToSample(int id, double timeout){
//         
//         actions.add(new MecanumAction(){
//             ElapsedTime timer;
//             int sampleX;
//             double yPow;
//             double thetaPow;
//             
//             @Override 
//             void start(){
//                 timer = new ElapsedTime();
//             }
//             
//             @Override 
//             void update(){
//                 
//                 sampleX = bot.getSampleX(id);
//                 
//                 if(lastAngle<-pid.odo.getHeading()) thetaPow = .08;
//                 else thetaPow = -.08;
//                 
//                 if(sampleX>180){
//                     //move right
//                     yPow = -.2;
//                 }
//                 else{
//                     //move left 
//                     yPow = .2;
//                 }
//                 
//                 bot.driveXYW(0, yPow, thetaPow);
//                 
//                 if(Math.abs(180-sampleX)<15 || timer.seconds()>timeout){
//                     bot.driveXYW(0, 0, 0);
//                     Pose2D curPos = pid.odo.getPosition();
//                     lastX = curPos.getX(DistanceUnit.INCH);
//                     lastY = curPos.getY(DistanceUnit.INCH);
//                     lastAngle = -curPos.getHeading(AngleUnit.DEGREES);
//                     nextState();
//                 } 
//                 
//                 
//             }
//         });
//     }
//     public void moveTo(double x, double y, double theta){
//         
//         actions.add(new MecanumAction(){
//             double tx = x;
//             double ty = y;
//             double tAngle = theta;
// 
// 
//             ElapsedTime timer;
//             double seconds = 4;
//             
//             @Override
//             void start(){
//                 if(tx == 999) tx = lastX;
//                 if(ty == 999) ty = lastY;
//                 if(tAngle == 999) tAngle = lastAngle;
//                 pid.moveTo(tx,ty,tAngle);
//                 lastX = tx;
//                 lastY = ty;
//                 lastAngle = tAngle;
//                 timer = new ElapsedTime();
//             }
//             @Override
//             void update(){
//                 pid.update();
//                 Pose2D curPos = pid.odo.getPosition();
//                 double deltaX = tx - curPos.getX(DistanceUnit.INCH);
//                 double deltaY = ty - curPos.getY(DistanceUnit.INCH);
//                 double deltaAngle = tAngle + curPos.getHeading(AngleUnit.DEGREES);
//                 deltaAngle = Mathf.angleWrap(deltaAngle);
//                 double dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//                 
//                 if(dist < 2 && Math.abs(deltaAngle) < 3){
//                     nextState();
//                 }
//                 
//             }
//         });
//     }
//     public void moveTo(double x, double y, double theta, Runnable command){
//         
//         actions.add(new MecanumAction(){
//             double tx = x;
//             double ty = y;
//             double tAngle = theta;
// 
// 
//             ElapsedTime timer;
//             double seconds = 2;
//             
//             @Override
//             void start(){
//                 pid.moveTo(tx,ty,tAngle);
//                 lastX = x;
//                 lastY = y;
//                 lastAngle = theta;
//                 timer = new ElapsedTime();
//                 command.run();
//             }
//             @Override
//             void update(){
//                 pid.update();
//                 Pose2D curPos = pid.odo.getPosition();
//                 double deltaX = tx - curPos.getX(DistanceUnit.INCH);
//                 double deltaY = ty - curPos.getY(DistanceUnit.INCH);
//                 double deltaAngle = tAngle + curPos.getHeading(AngleUnit.DEGREES);
//                 deltaAngle = Mathf.angleWrap(deltaAngle);
//                 double dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//                 if(timer.seconds() > seconds || dist < 1 && Math.abs(deltaAngle) < Math.toRadians(2)){
//                     nextState();
//                 }
//                 
//             }
//         });
//     }
//     public void moveTo(double x, double y, double theta, double timeout){
//         
//         actions.add(new MecanumAction(){
//             double tx = x;
//             double ty = y;
//             double tAngle = theta;
// 
// 
//             ElapsedTime timer;
//             double seconds = timeout;
//         
//             @Override
//             void start(){
//                 pid.moveTo(x,y,theta);
//                 lastX = x;
//                 lastY = y;
//                 lastAngle = theta;
//                 timer = new ElapsedTime();
//             }
//             @Override
//             void update(){
//                 pid.update();
//                 Pose2D curPos = pid.odo.getPosition();
//                 double deltaX = tx - curPos.getX(DistanceUnit.INCH);
//                 double deltaY = ty - curPos.getY(DistanceUnit.INCH);
//                 double deltaAngle = tAngle + curPos.getHeading(AngleUnit.DEGREES);
//                 deltaAngle = Mathf.angleWrap(deltaAngle);
//                 double dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//                 
//                 if(timer.seconds() > seconds || dist < 2 && Math.abs(deltaAngle) < Math.toRadians(2)){
//                     nextState();
//                 }
//                 
//             }
//         });
//     }
//     public void waitForSeconds(double seconds){
//         actions.add(new MecanumAction(){
//             ElapsedTime timer;
// 
//             @Override
//             void start(){
//                 pid.moveTo(lastX,lastY,lastAngle);
//                 timer = new ElapsedTime();
//             }
//             @Override
//             void update(){
//                 pid.update();
//                 if(timer.seconds() > seconds) nextState();
//             }
//         });
//     }
//     public void waitForSeconds(double seconds, Runnable command){
//         actions.add(new MecanumAction(){
//             ElapsedTime timer;
// 
//             @Override
//             void start(){
//                 command.run();
//                 pid.moveTo(lastX,lastY,lastAngle);
//                 timer = new ElapsedTime();
//             }
//             @Override
//             void update(){
//                 pid.update();
//                 if(timer.seconds() > seconds) nextState();
//             }
//         });
//     }
//     public void setMaxSpeed(double speed){
//         actions.add(new MecanumAction(){
//             @Override
//             void start(){
//                 pid.maxSpeed = speed;
//                 nextState();
//             }
//         });
//     }
//     public void nextState(){
//         if(current < actions.size()-1){
//             actions.get(current).end();
//             current++;
//             actions.get(current).start();
//         }
//         else{
//             actions.get(current).end();
//             current++;
//             actions.add(new MecanumAction(){
// 
//     
//                 @Override
//                 void start(){
//                     pid.moveTo(lastX, lastY, lastAngle);
//                 }
//                 @Override
//                 void update(){
//                     pid.update();
//                     
//                 }
//             });
//         }
//     }
// }
// 
// class MecanumAction{
//     void start(){};
//     void update(){};
//     void end(){};
// }
// 
// 
// 
// 
// 
// 
// 
// 
// 