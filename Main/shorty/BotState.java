package Main.shorty;

import Main.*;
import Main.auton.*;
import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class BotState{
    private static HashMap<String, BotState> states = new HashMap<>();
    public static BotState get(String s){
        try {
            return states.get(s);    
        } catch (Exception e) {
            return states.get("DEFAULT");
        }
        
    }
    String name;
    public BotState(String n){
        name = n;
        states.put(name, this);
    }
    public BotState(){
        name = "DEFAULT";
        states.put(name, this);
    }
    public BotState(String n, boolean bool){
        name = n;
        if(bool) states.put(name, this);
    }

    void start(){};
    void update(){};
    void end(){};
}