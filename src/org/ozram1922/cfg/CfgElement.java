package org.ozram1922.cfg;


import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class CfgElement
{
	public final Element mInternalElement;
	
	public CfgElement(Element element)
	{
		mInternalElement = element;
	}

	public CfgElement GetNthChild(String name, int num)
	{
		if(mInternalElement == null)
		{
			System.out.println("XML Element Became Null");
			return null;
		}
		
		NodeList children = mInternalElement.getElementsByTagName(name);
		if(num < 0 || children.getLength() < num - 1)
			return null;
		
		return new CfgElement((Element)children.item(num));
	}
	public void AppendChild(CfgElement child)
	{
		if(mInternalElement == null)
		{
			System.out.println("XML Element Became Null");
			return;
		}
		if(child == null)
		{
			return;
		}
		if(child.mInternalElement == null)
		{
			System.out.println("XML Element of Child is Null");
			return;
		}
		mInternalElement.appendChild(child.mInternalElement);
	}
	
	public float GetAttributeF(String name)
	{
		try
		{
			float ret = Float.parseFloat(GetAttribute(name));
			return ret;
		}
		catch(NumberFormatException ex)
		{
			ex.printStackTrace();
			return 0;
		}
	}
	public double GetAttributeD(String name)
	{
		try
		{
			double ret = Double.parseDouble(GetAttribute(name));
			return ret;
		}
		catch(NumberFormatException ex)
		{
			ex.printStackTrace();
			return 0;
		}
	}
	public int GetAttributeI(String name)
	{
		try
		{
			int ret = Integer.parseInt(GetAttribute(name));
			return ret;
		}
		catch(NumberFormatException ex)
		{
			ex.printStackTrace();
			return 0;
		}
	}
	public String GetAttribute(String name)
	{
		//we could return null, but blank is safer
		if(mInternalElement == null)
		{
			System.out.println("XML Element Became Null");
			return "";
		}
		return mInternalElement.getAttribute(name);
	}

	public void SetAttribute(String name, String value)
	{
		if(mInternalElement == null)
		{
			System.out.println("XML Element Became Null");
			return;
		}
		mInternalElement.setAttribute(name, value);
	}
	public void SetAttribute(String name, float value)
	{
		SetAttribute(name, Float.toString(value));
	}
	public void SetAttribute(String name, double value)
	{
		SetAttribute(name, Double.toString(value));		
	}
	public void SetAttribute(String name, int value)
	{
		SetAttribute(name, Integer.toString(value));
	}
	
	public boolean DeserializeChild(CfgInterface child)
	{
		CfgElement childElement = GetNthChild(child.GetElementTitle(), 0);
		if(childElement == null)
		{
			return false;
		}
		else
		{
			return child.Deserialize(childElement);
		}
	}
	
	public boolean SerializeChild(CfgInterface child, CfgDocument doc)
	{
		CfgElement childElement = doc.CreateElement(child.GetElementTitle());
		childElement = child.Serialize(childElement, doc);
		
		if(childElement == null)
		{
			return false;
		}
		else
		{
			AppendChild(childElement);
			return true;
		}
	}
}
