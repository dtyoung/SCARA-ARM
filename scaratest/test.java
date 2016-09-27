import ecs100.*;
import java.util.*;
import java.io.*;

/**
 * Write a description of class test here.
 * 
 * @author (your name) 
 * @version (a version number or a date)
 */
public class test {

    //     public static void main(String[] args) { 
    //         try{
    //             String command = "dir";
    // 
    //             Process proc = Runtime.getRuntime().exec(command);
    // 
    //             // Read the output
    // 
    //             BufferedReader reader =  
    //                 new BufferedReader(new InputStreamReader(proc.getInputStream()));
    // 
    //             String line = "";
    //             while((line = reader.readLine()) != null) {
    //                 System.out.print(line + "\n");
    //             }
    // 
    //             proc.waitFor();   
    //         }catch(Exception e){}
    //     }
    
    public static void main(String[] args) throws Exception {
        ProcessBuilder builder = new ProcessBuilder(
            "cmd.exe", "/c", "README.TXT");
        builder.redirectErrorStream(true);
        Process p = builder.start();
        BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()));
        String line;
        while (true) {
            line = r.readLine();
            if (line == null) { break; }
            System.out.println(line);
        }
    }
} 
