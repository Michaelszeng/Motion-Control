package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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
        this.path = path;
        buildPath(fileName);
    }

    public int size() {
        return path.size();
    }

    public ArrayList<PurePursuitPathPoint> buildPath(String xmlFileName) {
        importXMLPath(xmlFileName);
        return path1;
    }

    public String toString() {
        String output = "  x    y    h    isVertex\n";
        for (PurePursuitPathPoint pt : path1) {
            output = output + pt.x + ", " + pt.y + ", " + pt.h + ", " + pt.isVertex + "\n";
        }
        return output;
    }

    public static void joinSegments(ArrayList<PurePursuitPathPoint> path_2) throws Exception {
        //If they are the exact same point
        if (path1.get(path1.size() - 1).x == path_2.get(0).x && path1.get(path1.size() - 1).y == path_2.get(0).y && path1.get(path1.size() - 1).h == path_2.get(0).h && path1.get(path1.size() - 1).isVertex == path_2.get(0).isVertex) {
            //Append the 2nd path to the current path
            for (int i=1; i<path_2.size(); i++) {
                path1.add(path_2.get(i));
            }
        }
        else {
            throw new Exception("Cannot Join Paths. End and Start PurePursuitPathPoint are not the same.");
        }
    }

    public static void importXMLPath(String filename) {
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
            RobotLogger.dd(TAG, "path1.size: " + path1.size());
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
