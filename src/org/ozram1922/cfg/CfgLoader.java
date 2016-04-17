package org.ozram1922.cfg;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.Transformer;
import javax.xml.transform.Result;
import javax.xml.transform.Source;
import javax.xml.transform.stream.StreamResult;
import javax.xml.transform.dom.DOMSource;
import java.io.FileInputStream;
import java.io.StringReader;
import java.io.File;
import org.xml.sax.InputSource;
import java.util.ArrayList;
import org.w3c.dom.Element;
import org.w3c.dom.Document;
import org.w3c.dom.NodeList;

/*

Eventually this class will load a config file and create a graph tree of the XML values.  The can be loaded at run-time.

Also, it will be able to re-save values to the .xml files

It would be great if all of the classes could inherit from another class which has a method to get xml nodes from it overloaded by each class



*/
public class CfgLoader
{
  private ArrayList<CfgInterface> mCfgClasses;
  public CfgLoader()
  {
    mCfgClasses = new ArrayList<CfgInterface>();
  }

  public void RegisterCfgClass(CfgInterface cfgClass)
  {
    mCfgClasses.add(cfgClass);
  }

  //returns false ONLY if an exception is thown.  If the config file does not have complete data, it will not throw
  public boolean LoadFile(String data, boolean isFile)
  {
	//delete all of the 'things' before we load (to avoid GC not running in time)
	for(CfgInterface i : mCfgClasses)
	{
		if(i == null)
		{
			System.out.println("CfgInterface is Null");
			continue;
		}
		i.MakeCfgClassesNull();
	}
	
	//Collect garbage
	System.gc();  
	
    // create a new DocumentBuilderFactory
    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
    if(factory == null)
    {
    	System.out.println("Failed to Create Document Factory Builder when Deserializing Config");
    	return false;
    }

    try
    {
       // use the factory to create a documentbuilder
       DocumentBuilder builder = factory.newDocumentBuilder();
       if(builder == null)
       {
       		System.out.println("Failed to Create Document Builder when Deserializing Config");
       		return false;
       }
       
       Document doc = null;
       if(isFile)
       {
	       // create a new document from input source
	       doc = builder.parse(new InputSource(new FileInputStream(data)));
       }
       else
       {
    	   doc = builder.parse( new InputSource( new StringReader( data ) ) );  
       }
       if(doc == null)
       {
       		System.out.println("Failed to Create Document when Deserializing Config");
       		return false;
       }

       // get the first element
       Element rootElement = doc.getDocumentElement();
       if(rootElement == null)
       {
       		System.out.println("Failed to Root Element when Deserializing Config");
       		return false;
       }

       //iterate through the classes and try to find something to load from
       for(CfgInterface cfg : mCfgClasses)
       {
    	   if(cfg == null)
    	   {
    		   System.out.println("CfgInterface is Null when Deserializing Config");
    	   }
    	   //try to find a node with the ID of the class (this is how we know which node is which)
    	   //int id = cfg.GetID();

    	   String title = cfg.GetElementTitle();
    	   if(title == null)
    	   {
    		   System.out.println("Element Title String is Null");
    		   continue;
    	   }
         
    	   //get elements list
    	   NodeList elements = rootElement.getElementsByTagName(title);
    	   Element foundElement = null;

    	   if(elements.getLength() != 0)
	       {
    		   foundElement = (Element)elements.item(0);
	       }

    	   if(foundElement == null)
    	   {
    		   System.out.println("Loaded XML Element is Null");
    		   continue;
    	   }

    	   if(!cfg.Deserialize(new CfgElement(foundElement)))
    	   {
    		   System.out.println("Issues loading class: " + title);
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
    if(factory == null)
    {
    	System.out.println("Failed to Create Document Factory Builder when Serializing Config");
    	return false;
    }

    try
    {
       // use the factory to create a documentbuilder
       DocumentBuilder builder = factory.newDocumentBuilder();
       if(builder == null)
       {
    	   System.out.println("Failed to Create Document Factory when Serializing Config");
    	   return false;
       }

       // create a new document from input source
       //FileInputStream fis = new FileInputStream(filePath);
       //InputSource is = new InputSource(fis);
       Document doc = builder.newDocument();
       if(doc == null)
       {
    	   System.out.println("Failed to Create Document when Serializing Config");
    	   return false;
       }
       CfgDocument cfgDoc = new CfgDocument(doc);
       	
       // get the first element
       Element rootElement = doc.createElement("root");
       if(rootElement == null)
       {
    	   System.out.println("Failed to Create Root Elemenet When Serializing Config");
    	   return false;
       }

       //iterate through the classes and try to find something to load from
       for(CfgInterface cfg : mCfgClasses)
       {
    	   if(cfg == null)
    	   {
    		   System.out.println("Config Interface Class is Null");
    		   continue;
    	   }
    	   CfgElement blank = new CfgElement(doc.createElement(cfg.GetElementTitle()));
		   if(blank.mInternalElement == null)
		   {
    		   System.out.println("Failed to Create Element");
    		   continue;
		   }
    	   
    	   CfgElement element = cfg.Serialize(blank,cfgDoc);
    	   if(element == null)
    	   {
    		   System.out.println("CfgElement Returned from Serialize was Null");
    		   continue;
    	   }
           if(element.mInternalElement != null)
           {
        	   rootElement.appendChild(element.mInternalElement);
           }
       }

       //add the root element
       doc.appendChild(rootElement);

       //save the file
       Transformer transformer = TransformerFactory.newInstance().newTransformer();
       Result output = new StreamResult(new File(filePath));
       Source input = new DOMSource(doc);

       transformer.transform(input, output);

    } catch (Exception ex) {
       ex.printStackTrace();
       return false;
    }

    return true;


  }

}
