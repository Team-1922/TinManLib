package org.ozram1922.cfg;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.FileInputStream;
import org.xml.sax.InputSource;
import java.util.ArrayList;
import org.w3c.dom.Element;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/*

Eventually this class will load a config file and create a graph tree of the XML values.  The can be loaded at run-time.

Also, it will be able to re-save values to the .xml files

It would be great if all of the classes could inherit from another class which has a method to get xml nodes from it overloaded by each class



*/
public class CfgLoader
{
  private ArrayList<ConfigurableClass> mCfgClasses;
  public CfgLoader()
  {
    mCfgClasses = new ArrayList<ConfigurableClass>();
  }

  public void RegisterCfgClass(ConfigurableClass cfgClass)
  {
    mCfgClasses.add(cfgClass);
  }

  //returns false ONLY if an exception is thown.  If the config file does not have complete data, it will not throw
  public boolean LoadFile(String filePath)
  {
    // create a new DocumentBuilderFactory
    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();

    try
    {
       // use the factory to create a documentbuilder
       DocumentBuilder builder = factory.newDocumentBuilder();

       // create a new document from input source
       FileInputStream fis = new FileInputStream(filePath);
       InputSource is = new InputSource(fis);
       Document doc = builder.parse(is);

       // get the first element
       Element rootElement = doc.getDocumentElement();

       //iterate through the classes and try to find something to load from
       for(ConfigurableClass cfg : mCfgClasses)
       {
         //try to find a node with the ID of the class (this is how we know which node is which)
         int id = cfg.GetID();

         String title = cfg.GetElementTitle();

        //get elements list
         NodeList elements = rootElement.getElementsByTagName(title);
         Element foundElement = null;

         //find the one with the given ID
         for(int i = 0; i < elements.getLength(); ++i)
         {
           Element element = (Element)elements.item(i);
           int elementId = Integer.parseInt(element.getAttribute("id"));

           //is it the right element?
           if(elementId == id)
           {
             foundElement = element;
             break;
           }
         }

         //if(foundElement == null) DO NOTHING: handling this will be down to the class parsing

         if(!cfg.Deserialize(foundElement))
         {
           //let's pretend this does something useful :)
         }
       }

    } catch (Exception ex) {
       ex.printStackTrace();
       return false;
    }

    return true;
  }

  public boolean SaveFile(String filePath)
  {
    // create a new DocumentBuilderFactory
    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();

    try
    {
       // use the factory to create a documentbuilder
       DocumentBuilder builder = factory.newDocumentBuilder();

       // create a new document from input source
       //FileInputStream fis = new FileInputStream(filePath);
       //InputSource is = new InputSource(fis);
       Document doc = builder.newDocument();

       // get the first element
       Element rootElement = doc.createElement("root");

       //iterate through the classes and try to find something to load from
       for(ConfigurableClass cfg : mCfgClasses)
       {
           Element element = cfg.Serialize(doc);
           if(element != null)
           {
             rootElement.appendChild(element);
           }
       }

    } catch (Exception ex) {
       ex.printStackTrace();
       return false;
    }

    return true;


  }

}