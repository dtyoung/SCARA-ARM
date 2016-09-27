
/**
 * Class represents SCARA robotic arm.
 * 
 * @Arthur Roberts
 * @0.0
 */

import ecs100.UI;
import ecs100.UIFileChooser;
import java.awt.Color;
import java.util.*;
import java.io.*;

public class Arm
{

    // fixed arm parameters
    private int xm1;  // coordinates of the motor(measured in pixels of the picture)
    private int ym1;
    private int xm2;
    private int ym2;
    private double r;  // length of the upper/fore arm

    // parameters of servo motors - linear function pwm(angle)
    // each of two motors has unique function which should be measured
    // linear function cam be described by two points
    // motor 1, point1 
    private double pwm1_val_1; 
    private double theta1_val_1;
    // motor 1, point 2
    private double pwm1_val_2; 
    private double theta1_val_2;

    // motor 2, point 1
    private double pwm2_val_1; 
    private double theta2_val_1;
    // motor 2, point 2
    private double pwm2_val_2; 
    private double theta2_val_2;

    // current state of the arm
    private double theta1; // angle of the upper arm
    private double theta2;

    private double xj1;     // positions of the joints
    private double yj1; 
    private double xj2;
    private double yj2; 
    private double xt;     // position of the tool
    private double yt;
    private boolean valid_state; // is state of the arm physically possible?

    //Used for calibration
    List<Integer> motor1Pwm = new ArrayList<Integer>();
    List<Integer> motor1Angle = new ArrayList<Integer>();
    List<Integer> motor2Pwm = new ArrayList<Integer>();
    List<Integer> motor2Angle = new ArrayList<Integer>();
    
    //Equation in the form y = mx + b (angle = m*pwm + b)
    double m1Slope; //Slope of the line of best fit for motor 1
    double m1B; //Y intercept
    
    double m2Slope;//Slope of the line of best fit for motor 2
    double m2B;//y intercept
    //End of Calibration variables
    
    /**
     * Constructor for objects of class Arm
     */
    public Arm()
    {
        xm1 = 287; // set motor coordinates
        ym1 = 374;
        xm2 = 377;
        ym2 = 374;
        r = 154.0;
        theta1 = -90.0*Math.PI/180.0; // initial angles of the upper arms
        theta2 = -90.0*Math.PI/180.0;
        valid_state = false;
    }

    // draws arm on the canvas
    public void draw() //Completed already no need to edit
    {
        // draw arm
        int height = UI.getCanvasHeight();
        int width = UI.getCanvasWidth();
        // calculate joint positions
        xj1 = xm1 + r*Math.cos(theta1);
        yj1 = ym1 + r*Math.sin(theta1);
        xj2 = xm2 + r*Math.cos(theta2);
        yj2 = ym2 + r*Math.sin(theta2);

        //draw motors and write angles
        int mr = 20;
        UI.setLineWidth(5);
        UI.setColor(Color.BLUE);
        UI.drawOval(xm1-mr/2,ym1-mr/2,mr,mr);
        UI.drawOval(xm2-mr/2,ym2-mr/2,mr,mr);
        // write parameters of first motor
        String out_str=String.format("t1=%3.1f",theta1*180/Math.PI);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+2*mr);
        out_str=String.format("xm1=%d",xm1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+3*mr);
        out_str=String.format("ym1=%d",ym1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+4*mr);
        // ditto for second motor                
        out_str = String.format("t2=%3.1f",theta2*180/Math.PI);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+2*mr);
        out_str=String.format("xm2=%d",xm2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+3*mr);
        out_str=String.format("ym2=%d",ym2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+4*mr);
        // draw Field Of View
        UI.setColor(Color.GRAY);
        UI.drawRect(0,0,640,480);

        // it can be uncommented later when
        // kinematic equations are derived
        if ( valid_state) {
            // draw upper arms
            UI.setColor(Color.GREEN);
            UI.drawLine(xm1,ym1,xj1,yj1);
            UI.drawLine(xm2,ym2,xj2,yj2);
            //draw forearms
            UI.drawLine(xj1,yj1,xt,yt);
            UI.drawLine(xj2,yj2,xt,yt);
            // draw tool
            double rt = 20;
            UI.drawOval(xt-rt/2,yt-rt/2,rt,rt);
        }

    }

    // calculate tool position from motor angles 
    // updates variable in the class
    public void directKinematic(){

        // midpoint between joints
        double  xa = xj1 + 0.5*(xj2-xj1);
        double  ya = yj1 + 0.5*(yj2-yj1) ;
        // half distance between joints
        double d = 0.5*Math.sqrt((xj2-xj1)*(xj2-xj1) + (yj2-yj1)*(yj2-yj1));
        if (d<2*r){
            valid_state = true;
            // half distance between tool positions
            double  h = Math.sqrt(r*r-(d*d));
            double alpha= Math.atan((yj1-yj2)/(xj1-xj2));
            // tool position
            double xt = xa + h*Math.cos(Math.PI/2.0-alpha);
            double yt = ya + h*Math.sin(Math.PI/2.0-alpha);
            double xt2 = xa - h*Math.cos(Math.PI/2.0-alpha);
            double yt2 = ya - h*Math.sin(Math.PI/2.0-alpha);
        } else {
            valid_state = false;
        }

    }

    // motor angles from tool position
    // updates variables of the class
    public void inverseKinematic(double xt_new,double yt_new){

        valid_state = true;
        xt = xt_new;
        yt = yt_new;
        valid_state = true;

        // distance between pen and motor1        
        double dx1 = xt - xm1; 
        double dy1 = yt - ym1;
        double d1 = Math.sqrt(dx1*dx1 + dy1*dy1);

        if (d1>2*r){ //If the distance between the pen and the motor is greater than the possible length of the 2 arms
            UI.println("Arm 1 - can not reach");
            valid_state = false;
            return;
        }

        double l1 = d1/2; //Half the distance between the pen and motor
        double h1 = Math.sqrt(r*r - l1*l1); 
        // elbows positions, elbows facing out

        double thetaA = Math.atan2(yt-ym1, xt-xm1);
        xj1 = xm1 + (xt-xm1)/2 -  h1*Math.cos((Math.PI/2.0)+thetaA);
        yj1 = ym1 + (yt-ym1)/2 - h1*Math.sin((Math.PI/2.0) + thetaA);

        theta1 = Math.atan2(yj1-ym1, xj1-xm1); //angle of motor 1

       
        if ((theta1>0)||(theta1<-Math.PI)){
            valid_state = false;
            UI.println("Ange 1 -invalid");
             return;
        }

        theta2 = Math.atan2(yj2 - ym1,xj2-xm1);
        // distance between pen and motor 2
        double dx2 = xt - xm2;
        double dy2 = yt - ym2;
        double d2 = Math.sqrt(dx2*dx2 + dy2*dy2);
        if (d2>2*r){
            UI.println("Arm 2 - can not reach");
            valid_state = false;
            return;
        }

        double l2 = d2/2;

        double h2 = Math.sqrt(r*r - d2*d2/4);

        //double thetaA = Math.asin(d2/(yt-ym2));
        double thetaB = Math.atan2(yt-ym2, xt-xm2);
        // elbows positions

        xj2 = xm2 + 0.5*(xt - xm2) - h2*Math.cos(thetaB - Math.PI/2.0 );
        yj2 = ym2 + 0.5*(yt - ym2) - h2*Math.sin(thetaB - Math.PI/2.0 );
        
        double dj = Math.sqrt((xj2 - xj1)*(xj2 - xj1) + (yj2 - yj1)*(yj2 - yj1));
        if(dj > 2*r){
            valid _state = false;
            UI.println("Joints too far apart!");
            return;
        }
        
        // motor angles for both 1st elbow positions
        theta2 =  Math.atan2((yj2-ym2), (xj2-xm2));
        if ((theta2>0)||(theta2<-Math.PI)){
            valid_state = false;
            UI.println("Ange 2 -invalid");
            return;
        }

        UI.printf("xt:%3.1f, yt:%3.1f\n",xt,yt);
        UI.printf("theta1:%3.1f, theta2:%3.1f\n",theta1*180/Math.PI,theta2*180/Math.PI);
        return;
    }

    // returns angle of motor 1
    public double get_theta1(){
        return theta1;
    }
    // returns angle of motor 2
    public double get_theta2(){
        return theta2;
    }
    // sets angle of the motors
    public void set_angles(double t1, double t2){
        theta1 = t1;
        theta2 = t2;
    }

    // returns motor control signals
    // for motor to be in position(angle) theta1
    // linear intepolation
    public int get_pwm1(){
        int pwm = (int)((theta1 - m1B)/m1Slope);
        return pwm;
    }
    // ditto for motor 2
    public int get_pwm2(){
        int pwm =(int)((theta2 - m2B)/m2Slope);
        //pwm = (int)(pwm2_90 + (theta2 - 90)*pwm2_slope); Was here previously
        return pwm;
    }
    
    //Calibration Methods
    
    //Calibrates the arm
    //Sets the values of motor1Slope, m1B, motor2Slope, m2B
    public void calibrate(){
        String calibrateFile = UIFileChooser.open("Select Calibration File");
        initializeLists(calibrateFile);
        m1Fit();
        m2Fit();
    }
    
    //Takes a file with a paste from the calibration display on the PI.
    //Must include both ***Motor1 headers
    //Adds each angle and pwm into the appropriate lists
    public void initializeLists(String fname){
        String line;
        String token;
        int index; //Index of '='
        int index2; //Index of '.'
        boolean motor1 = true;
        try{
            Scanner scan = new Scanner(new File(fname));
            scan.nextLine();
            while(scan.hasNextLine()){
                token = scan.next();
                if(token.contains("***")){
                    motor1 = false;
                    scan.nextLine();
                }
                else{
                    if(motor1){
                        index = token.indexOf("=");
                        motor1Pwm.add(Integer.parseInt(token.substring(index + 1)));
                        scan.next();scan.next();
                        token = scan.next();
                        index = token.indexOf("=");
                        index2 = token.indexOf(".");
                        motor1Angle.add(Integer.parseInt(token.substring(index + 1, index2)));
                        scan.next();
                    }
                    else{
                        token = scan.next();
                        index = token.indexOf("=");
                        motor2Pwm.add(Integer.parseInt(token.substring(index + 1)));
                        scan.next();scan.next();
                        token = scan.next();
                        index = token.indexOf("=");
                        index2 = token.indexOf(".");
                        motor2Angle.add(Integer.parseInt(token.substring(index + 1, index2)));
                    }
                }
            }
        }
        catch(Exception e){UI.println("Invalid File: " + e);}
    }
    
    public void m1Fit(){
        double meanPwm = 0;
        double meanAngle = 0;
        for(int counter = 0; counter < motor1Pwm.size(); counter++){
            meanPwm += motor1Pwm.get(counter);
        }
        meanPwm /= motor1Pwm.size();
        UI.println("MEAN PWM: " + meanPwm);
        
        for(int counter = 0; counter < motor1Angle.size(); counter++){
            meanAngle += motor1Angle.get(counter);
        }
        meanAngle /= motor1Angle.size();
        UI.println("MEAN angle: " + meanAngle);
        
        double total = 0;
        int div = 0;
        for(int counter = 0; counter < motor1Angle.size(); counter++){
            total += (motor1Pwm.get(counter) - meanPwm)*(motor1Angle.get(counter) - meanAngle);
            div += Math.pow((motor1Pwm.get(counter) - meanPwm),2);
        }
        
        m1Slope = total/div;
        m1B = meanAngle - (m1Slope*meanPwm);
        UI.println("Equation: y = " + m1Slope + "x + " + m1B);
    }
    
    public void m2Fit(){
        double meanPwm = 0;
        double meanAngle = 0;
        for(int counter = 0; counter < motor2Pwm.size(); counter++){
            meanPwm += motor2Pwm.get(counter);
        }
        meanPwm /= motor2Pwm.size();
        UI.println("MEAN PWM: " + meanPwm);
        
        for(int counter = 0; counter < motor2Angle.size(); counter++){
            meanAngle += motor2Angle.get(counter);
        }
        meanAngle /= motor2Angle.size();
        UI.println("MEAN angle: " + meanAngle);
        
        double total = 0;
        int div = 0;
        for(int counter = 0; counter < motor2Angle.size(); counter++){
            total += (motor2Pwm.get(counter) - meanPwm)*(motor2Angle.get(counter) - meanAngle);
            div += Math.pow((motor2Pwm.get(counter) - meanPwm),2);
        }
        
        m2Slope = total/div;
        m2B = meanAngle - (m2Slope*meanPwm);
        UI.println("Equation: y = " + m2Slope + "x + " + m2B);
    }
}
