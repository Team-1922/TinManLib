package org.ozram1922.cfg;

import org.w3c.dom.Element;
import org.w3c.dom.Document;

/*

This interface should be inherited by all classes which want to utilize the XML configuration loading and saving

*/

public abstract class ConfigurableClass
{
  private Element tmpElement = null;
  private String mUniqueName;

  public ConfigurableClass(String uniqueName)
  {
    mUniqueName = uniqueName;
  }
  public boolean Deserialize(Element thisElement)
  {
    if(thisElement == null)
      return false;

    tmpElement = thisElement;
    return DeserializeInternal();
  }
  protected abstract boolean DeserializeInternal();
  public Element Serialize(Document doc)
  {
    if(doc == null)
      return null;

    tmpElement = doc.createElement(mUniqueName);
    SerializeInternal(doc);
    return tmpElement;
  }
  protected abstract void SerializeInternal(Document doc);
  public String GetElementTitle()
  {
    return mUniqueName;
  }

  /*

  These are helper functions for adding attributes and children (all 'name' values must be UNIQUE)

  */

  protected void SetAttribute(String name, String value)
  {
    tmpElement.setAttribute(name, value);
  }
  protected String GetAttribute(String name)
  {
    return tmpElement.getAttribute(name);
  }
  protected void AddChild(Element childElement)
  {
    tmpElement.appendChild(childElement);
  }
}
