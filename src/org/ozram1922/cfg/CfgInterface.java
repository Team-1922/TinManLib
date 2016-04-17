package org.ozram1922.cfg;



public interface CfgInterface
{
	boolean Deserialize(CfgElement element);
  
	//'blank' is a pre-created blank document; 'doc' is used to create elements and add them to the blank document
	CfgElement Serialize(CfgElement blank, CfgDocument doc);
	
	//Don't call 'MakeCfgClassesNull()', only for internal use
	//This is used so garbage collection can be called beforehand
	void MakeCfgClassesNull();
	
	//this returns the string the loader is looking for in the XML element tags
	String GetElementTitle();
}
