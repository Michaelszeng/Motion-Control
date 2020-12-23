package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.w3c.dom.DOMException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class PurePursuitPath {
    public static String TAG = "PurePursuitPath";

    public ArrayList<Pose2d> path;
    public static ArrayList<PurePursuitPathPoint> path1 = new ArrayList<>();

    public PurePursuitPath(ArrayList<Pose2d> path) {
        this.path = path;
    }

    public PurePursuitPath(String fileName) {
        path1 = buildPath(fileName);
    }

    public PurePursuitPath(ArrayList<Pose2d> path, String fileName) {
        RobotLogger.dd(TAG, "Path File Name: " + fileName);
        this.path = path;
        buildPath(fileName);
        int i=1;
        while (i < path1.size()) {
            if (path1.get(i).x == path1.get(i-1).x && path1.get(i).y == path1.get(i-1).y && path1.get(i).h == path1.get(i-1).h) {
                path1.remove(i);
            }
            else {
                i++;
            }
        }
    }

    public int size() {
        return path.size();
    }

    public ArrayList<PurePursuitPathPoint> buildPath(String xmlFileName) {
        importXMLPath(xmlFileName);
        return path1;
    }

    public String toString() {
        String netOutput = "";
        String output = "";
        int counterFrom0 = 0;
        int counter = 0;
        for (PurePursuitPathPoint pt : path1) {
            output = output + pt.toString() + "\n";
            netOutput = netOutput + pt.toString() + "\n";
            if (counter == 15 || counterFrom0 == path1.size()-1) {
                RobotLogger.dd(TAG, output);
                output = "";
                counter = 0;
            }
            counter++;
            counterFrom0++;
        }
        return netOutput;
    }

    public static void importXMLPath(String filename) {
        path1.clear();
        String full_path = AppUtil.CONFIG_FILES_DIR + "/" + filename;
        try {
            File inputFile = new File(full_path);
            DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
            DocumentBuilder db = dbf.newDocumentBuilder();
            Document doc = db.parse(inputFile);
            doc.getDocumentElement().normalize();
            NodeList nList = doc.getElementsByTagName("Pose");
//            RobotLogger.dd(TAG, "nList.getLength(): " + nList.getLength());
            for (int i = 0; i < nList.getLength(); i++) {
                Node nNode = nList.item(i);
                if (nNode.getNodeType() == Node.ELEMENT_NODE) {
                    Element eElement = (Element) nNode;
                    String xStr = eElement.getElementsByTagName("x").item(0).getTextContent();
                    String yStr = eElement.getElementsByTagName("y").item(0).getTextContent();
                    String hStr = eElement.getElementsByTagName("heading").item(0).getTextContent();
                    String isVertexStr = eElement.getElementsByTagName("isVertex").item(0).getTextContent();
                    double x = Double.parseDouble(xStr);
                    double y = Double.parseDouble(yStr);
                    double hDegrees = Double.parseDouble(hStr);
                    double h = (hDegrees*Math.PI)/180;
                    boolean isVertex = Boolean.parseBoolean(isVertexStr);
                    PurePursuitPathPoint pt = new PurePursuitPathPoint(x, y, h, isVertex);
                    path1.add(pt);
                }
            }
//            RobotLogger.dd(TAG, "path1.size: " + path1.size());
        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (DOMException e) {
            e.printStackTrace();
        }
    }

}
