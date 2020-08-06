package org.firstinspires.ftc.teamcode.ML_Auto.Utils;

import android.content.Context;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStreamWriter;

public class FileWriter {
    public static void writeSerializedObject(String filePath, Object data) {
        try {
            File folder = new File(filePath.substring(0, filePath.lastIndexOf("/") + 1));
            new File(filePath).delete();
            ObjectOutputStream outputStreamWriter = new ObjectOutputStream(new FileOutputStream(new File(filePath), true));
            outputStreamWriter.writeObject(data);
            outputStreamWriter.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static Object getSerializedObject(String filePath){
        Object output = null;
        try {
            File folder = new File(filePath.substring(0, filePath.lastIndexOf("/") + 1));
            folder.mkdirs();
            ObjectInputStream outputStreamReader = new ObjectInputStream(new FileInputStream(new File(filePath)));
            output = outputStreamReader.readObject();
            outputStreamReader.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return output;
    }

    public static void writeFile(String filePath, String data, boolean append) {
        try {
            File folder = new File(filePath.substring(0,filePath.lastIndexOf("/") + 1));
            folder.mkdirs();
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(new File(filePath), append));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

}
