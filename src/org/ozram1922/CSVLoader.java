package org.ozram1922;


import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Stream;

public class CSVLoader
{
  static public ArrayList<ArrayList<Double>> LoadCSVFile(Path filePath) throws IOException
  {
    List<String> lineList;
    try {
      lineList =  Files.readAllLines(filePath);

      System.out.println(lineList);

    } catch (IOException e) {
      e.printStackTrace();
      throw e;
    }

    //break up into an array of arrays
    ArrayList<ArrayList<Double>> dataPoints = new ArrayList<ArrayList<Double>>();
    for (String currLine : lineList)
    {
      //separate each line into an array
      String[] parts = currLine.split(",");

      ArrayList<Double> rowData = new ArrayList<Double>();
      for(String val : parts)
      {
        rowData.add(Double.parseDouble(val));
      }
      dataPoints.add(rowData);
    }

    return dataPoints;
  }
}
