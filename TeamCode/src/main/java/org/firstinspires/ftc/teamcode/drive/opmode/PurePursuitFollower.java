package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;
import org.w3c.dom.Document;
import org.w3c.dom.NodeList;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

/*
 * Follows a path using pure pursuit algorithm
 */
@Config
@Autonomous(name = "PurePursuitFollower", group = "drive")

public class PurePursuitFollower extends LinearOpMode {
    private static String TAG = "PurePursuitFollower";
    static String pathFileName = "path.xml";
    FtcDashboard dashboard;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        packet.put("hi", "hi");
        dashboard.sendTelemetryPacket(packet);

        String pathString = "79,635,79,635,81,99,79,630,79,625,79,620,79,615,79,610,79,605,79,600,79,595,79,590,79,585,79,580,79,575,79,570,79,565,79,560,79,555,79,550,79,545,79,540,79,535,79,530,79,525,79,520,79,515,79,510,79,505,79,500,79,495,79,490,79,485,79,480,79,475,79,470,79,465,79,460,79,455,79,450,79,445,79,440,79,435,79,430,79,425,79,420,79,415,79,410,79,405,79,400,79,395,79,390,79,385,79,380,79,375,79,370,80,365,80,360,80,355,80,350,80,345,80,340,80,335,80,330,80,325,80,320,80,315,80,310,80,305,80,300,80,295,80,290,80,285,80,280,80,275,80,270,80,265,80,260,80,255,80,250,80,245,80,240,80,235,80,230,80,225,80,220,80,215,80,210,80,205,80,200,80,195,80,190,80,185,80,180,80,175,80,170,80,165,80,160,80,155,80,150,80,145,80,140,80,135,80,130,80,125,80,120,80,115,80,110,80,105,80,100,79,635,81,99,79,630,79,625,79,620,79,615,79,610,79,605,79,600,79,595,79,590,79,585,79,580,79,575,79,570,79,565,79,560,79,555,79,550,79,545,79,540,79,535,79,530,79,525,79,520,79,515,79,510,79,505,79,500,79,495,79,490,79,485,79,480,79,475,79,470,79,465,79,460,79,455,79,450,79,445,79,440,79,435,79,430,79,425,79,420,79,415,79,410,79,405,79,400,79,395,79,390,79,385,79,380,79,375,79,370,80,365,80,360,80,355,80,350,80,345,80,340,80,335,80,330,80,325,80,320,80,315,80,310,80,305,80,300,80,295,80,290,80,285,80,280,80,275,80,270,80,265,80,260,80,255,80,250,80,245,80,240,80,235,80,230,80,225,80,220,80,215,80,210,80,205,80,200,80,195,80,190,80,185,80,180,80,175,80,170,80,165,80,160,80,155,80,150,80,145,80,140,80,135,80,130,80,125,80,120,80,115,80,110,80,105,80,100,622,97,85,98,90,98,95,98,100,98,105,98,110,98,115,98,120,98,125,98,130,98,135,98,140,98,145,98,150,98,155,98,160,98,165,98,170,98,175,98,180,98,185,98,190,98,195,98,200,98,205,98,210,98,215,98,220,98,225,98,230,98,235,98,240,98,245,98,250,98,255,98,260,98,265,98,270,98,275,98,280,98,285,98,290,98,295,98,300,98,305,98,310,98,315,98,320,98,325,98,330,98,335,98,340,98,345,98,350,98,355,97,360,97,365,97,370,97,375,97,380,97,385,97,390,97,395,97,400,97,405,97,410,97,415,97,420,97,425,97,430,97,435,97,440,97,445,97,450,97,455,97,460,97,465,97,470,97,475,97,480,97,485,97,490,97,495,97,500,97,505,97,510,97,515,97,520,97,525,97,530,97,535,97,540,97,545,97,550,97,555,97,560,97,565,97,570,97,575,97,580,97,585,97,590,97,595,97,600,97,605,97,610,97,615,97,620,97,79,635,81,99,79,630,79,625,79,620,79,615,79,610,79,605,79,600,79,595,79,590,79,585,79,580,79,575,79,570,79,565,79,560,79,555,79,550,79,545,79,540,79,535,79,530,79,525,79,520,79,515,79,510,79,505,79,500,79,495,79,490,79,485,79,480,79,475,79,470,79,465,79,460,79,455,79,450,79,445,79,440,79,435,79,430,79,425,79,420,79,415,79,410,79,405,79,400,79,395,79,390,79,385,79,380,79,375,79,370,80,365,80,360,80,355,80,350,80,345,80,340,80,335,80,330,80,325,80,320,80,315,80,310,80,305,80,300,80,295,80,290,80,285,80,280,80,275,80,270,80,265,80,260,80,255,80,250,80,245,80,240,80,235,80,230,80,225,80,220,80,215,80,210,80,205,80,200,80,195,80,190,80,185,80,180,80,175,80,170,80,165,80,160,80,155,80,150,80,145,80,140,80,135,80,130,80,125,80,120,80,115,80,110,80,105,80,100,622,97,85,98,90,98,95,98,100,98,105,98,110,98,115,98,120,98,125,98,130,98,135,98,140,98,145,98,150,98,155,98,160,98,165,98,170,98,175,98,180,98,185,98,190,98,195,98,200,98,205,98,210,98,215,98,220,98,225,98,230,98,235,98,240,98,245,98,250,98,255,98,260,98,265,98,270,98,275,98,280,98,285,98,290,98,295,98,300,98,305,98,310,98,315,98,320,98,325,98,330,98,335,98,340,98,345,98,350,98,355,97,360,97,365,97,370,97,375,97,380,97,385,97,390,97,395,97,400,97,405,97,410,97,415,97,420,97,425,97,430,97,435,97,440,97,445,97,450,97,455,97,460,97,465,97,470,97,475,97,480,97,485,97,490,97,495,97,500,97,505,97,510,97,515,97,520,97,525,97,530,97,535,97,540,97,545,97,550,97,555,97,560,97,565,97,570,97,575,97,580,97,585,97,590,97,595,97,600,97,605,97,610,97,615,97,620,97,85,638,618,100,614,104,611,107,607,111,604,114,600,118,597,121,593,125,590,128,586,132,583,136,579,139,576,143,572,146,569,150,565,153,562,157,558,160,555,164,551,167,548,171,544,175,540,178,537,182,533,185,530,189,526,192,523,196,519,199,516,203,512,207,509,210,505,214,502,217,498,221,495,224,491,228,488,231,484,235,481,238,477,242,474,246,470,249,467,253,463,256,459,260,456,263,452,267,449,270,445,274,442,277,438,281,435,285,431,288,428,292,424,295,421,299,417,302,414,306,410,309,407,313,403,317,400,320,396,324,393,327,389,331,385,334,382,338,378,341,375,345,371,348,368,352,364,356,361,359,357,363,354,366,350,370,347,373,343,377,340,380,336,384,333,387,329,391,326,395,322,398,319,402,315,405,312,409,308,412,304,416,301,419,297,423,294,427,290,430,287,434,283,437,280,441,276,444,273,448,269,451,266,455,262,458,259,462,255,466,252,469,248,473,245,476,241,480,238,483,234,487,231,490,227,494,223,497,220,501,216,505,213,508,209,512,206,515,202,519,199,522,195,526,192,529,188,533,185,537,181,540,178,544,174,547,171,551,167,554,164,558,160,561,157,565,153,568,149,572,146,576,142,579,139,583,135,586,132,590,128,593,125,597,121,600,118,604,114,608,111,611,107,615,104,618,100,622,97,625,93,629,90,632,86,636,";
        ArrayList<ArrayList<Integer>> path = getPathTemp(pathString);
        ArrayList<Integer> pathX = path.get(0);
        ArrayList<Integer> pathY = path.get(1);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
//        for (int i : pathX) {
//            Log.d(TAG, "coord x: " + String.valueOf(i));
//        }
//        for (int i : pathY) {
//            Log.d(TAG, "coord y: " + String.valueOf(i));
//        }



        waitForStart();
        Log.d(TAG, "Program Started");
    }

    public static ArrayList<ArrayList<Integer>> getPathTemp(String coords) {
        ArrayList<ArrayList<Integer>> path = new ArrayList<>();  //will have 2 arraylists in it, 1st one has x coords, second has y coords
        ArrayList<String> allCoordinates = new ArrayList<>(Arrays.asList(coords.split(",")));     //x, y, x, y, x, y ...
        ArrayList<Integer> xCoords = new ArrayList<>();
        ArrayList<Integer> yCoords = new ArrayList<>();
        for (int i=0; i<allCoordinates.size(); i++) {
            if (i % 2 == 0) {   //x coord
                Log.d(TAG, String.valueOf(allCoordinates.get(i)));
                xCoords.add(Integer.parseInt(allCoordinates.get(i)));
            }
            else {      //y coord
                Log.d(TAG, "----------------------" + String.valueOf(allCoordinates.get(i)));
                yCoords.add(Integer.parseInt(allCoordinates.get(i)));
            }
        }
        path.add(xCoords);
        path.add(yCoords);
        return path;
    }

    public static ArrayList<ArrayList<Integer>> getPath(String filename) {
        ArrayList<ArrayList<Integer>> path = new ArrayList<>();  //will have 2 arraylists in it, 1st one has x coords, second has y coords
        String coords = "";

//        try {
//            BufferedReader br = new BufferedReader(new FileReader("C:\\Users\\Michael Zeng\\Documents\\FTC\\road-runner-quickstart\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\drive\\opmode\\path.txt"));
//            String line = br.readLine();
//            while(line != null){
//                coords = line;
//                line = br.readLine();
//            }
//            br.close();
//        } catch (FileNotFoundException e1) {
//            // TODO Auto-generated catch block
//            e1.printStackTrace();
//        } catch (IOException e) {
//            // TODO Auto-generated catch block
//            e.printStackTrace();
//        }

        String full_path = AppUtil.CONFIG_FILES_DIR + "/" + filename;
        try {
            File inputFile = new File(AppUtil.CONFIG_FILES_DIR+"/"+pathFileName);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(inputFile);
            doc.getDocumentElement().normalize();
            RobotLogger.dd(TAG, "Root element :" + doc.getDocumentElement().getNodeName());
            NodeList nodeList = doc.getElementsByTagName("coordinates");

            RobotLogger.dd(TAG, "NodeList: " + nodeList);
        } catch (Exception e) {
            RobotLogger.dd(TAG, "cannot find path file: " + full_path);
            e.printStackTrace();
        }


        ArrayList<String> allCoordinates = new ArrayList<>(Arrays.asList(coords.split(",")));     //x, y, x, y, x, y ...
        ArrayList<Integer> xCoords = new ArrayList<>();
        ArrayList<Integer> yCoords = new ArrayList<>();
        for (int i=0; i<allCoordinates.size(); i++) {
            if (i % 2 == 0) {   //x coord
                xCoords.add(Integer.parseInt(allCoordinates.get(i)));
            }
            else {      //y coord
                Log.d(TAG, "y: " + String.valueOf(i));
                yCoords.add(Integer.parseInt(allCoordinates.get(i)));
            }
        }
        path.add(xCoords);
        path.add(yCoords);
        return path;
    }
}
