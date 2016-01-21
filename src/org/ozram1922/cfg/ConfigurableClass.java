package org.ozram1922.cfg;

import org.w3c.dom.Element;
import org.w3c.dom.Document;

/*

This interface should be inherited by all classes which want to utilize the XML configuration loading and saving

*/

public abstract class ConfigurableClass
{
  public abstract boolean Deserialize(Element parentNode);
  public Element Serialize(Document doc)
  {
    Element thisElement = doc.createElement(GetElementTitle());
    thisElement.setAttribute("id", Integer.toString(GetID()));
    return SerializeInternal(doc, thisElement);
  }
  protected abstract Element SerializeInternal(Document doc, Element thisElement);
  public abstract int GetID();
  public abstract String GetElementTitle();
}
