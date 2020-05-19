package org.agents;

import java.io.Serializable;

public enum Color implements Serializable {
	Blue, 
	Red,
	Cyan,
	Purple,
	Green,
	Orange,
	Pink,
	Grey,
	Lightblue,
	Brown;

	public static Color geFromName(String color_name)
	{
		switch (color_name)
		{
		case "blue":			return Color.Blue;
		case "red":				return Color.Red;
		case "cyan":			return Color.Cyan;
		case "purple":			return Color.Purple;
		case "green":			return Color.Green;
		case "orange":			return Color.Orange;
		case "pink":			return Color.Pink;
		case "grey":			return Color.Grey;
		case "lightblue":		return Color.Lightblue;
		case "brown":			return Color.Brown;

		default: 			return Color.Lightblue;
		}
	}
	
	public static Color getFrom(int color_code)
	{
		switch (color_code)
		{
		case 1:  return Color.Blue;
		case 2:  return Color.Red;
		case 3:  return Color.Cyan;
		case 4:  return Color.Purple;
		case 5:  return Color.Green;
		case 6:  return Color.Orange;
		case 7:  return Color.Pink;
		case 8:  return Color.Grey;
		case 9: return Color.Lightblue;
		case 10: return Color.Brown;

		default: 	return Color.Lightblue;
		}
	}
	
	public static int getColorCoded(Color color)
	{
		switch (color)
		{
		case Blue: 		return 1;
		case Red:     	return 2;
		case Cyan:    	return 3;
		case Purple:  	return 4;
		case Green: 	return 5;
		case Orange:	return 6;
		case Pink:		return 7;
		case Grey: 		return 8;
		case Lightblue: return 9;
		case Brown: 	return 10;

		default: 	return 9;

		}
	}
}
