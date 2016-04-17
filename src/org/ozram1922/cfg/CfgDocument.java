package org.ozram1922.cfg;

import org.w3c.dom.Document;

public class CfgDocument 
{
	public final Document mInternalDoc;

	public CfgDocument(Document doc)
	{
		mInternalDoc = doc;
	}
	
	public CfgElement CreateElement(String name)
	{
		if(mInternalDoc == null)
		{
			System.out.println("XML Document Became Null");
			return null;
		}
		return new CfgElement(mInternalDoc.createElement(name));
	}
}
