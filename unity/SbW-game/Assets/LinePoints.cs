using UnityEngine;
using System.Collections;
using System.IO;

public class LinePoints : MonoBehaviour
{	
   static int numOfPoints = 161; //160
   static int numOfReferenceLines = 1931; //2321
	string[] allReference = new string[numOfReferenceLines];
   			       
    void Start() 
    {
		LineRenderer lineRenderer = GetComponent<LineRenderer>();
		string referenceFileLocation = Path.Combine(Application.streamingAssetsPath, "reference_strings.txt");
		StreamReader reader = new StreamReader(referenceFileLocation);
		// Read the file and save all reference path
    	for (int i = 0; i < numOfReferenceLines; i++) 
    	{
       		string refString = reader.ReadLine();
			allReference[i] = refString;
    	}
		// Extract and render the reference at t=0
		string[] xOfPoints = allReference[0].Trim().Split(',');
    	Vector3[] linePoints = new Vector3[numOfPoints];
    	for (int i = 0; i < numOfPoints; i++) 
    	{
			float y = ( 1f / (float) (numOfPoints-1) ) * (float) i;
        	float x = float.Parse(xOfPoints[i]);
        	linePoints[i].Set(x,y,0.0f);
    	}
		lineRenderer.positionCount = linePoints.Length;
    	lineRenderer.SetPositions(linePoints);
    }
    
    void Update()
    {
		LineRenderer lineRenderer = GetComponent<LineRenderer>();
		int currentLine = GameObject.Find("UDPComms").GetComponent<UDPReceive>().currLine;
    	// Extract and render the reference at current time
		string[] xOfPoints = allReference[currentLine].Trim().Split(',');
    	Vector3[] linePoints = new Vector3[numOfPoints];
    	for (int i = 0; i < numOfPoints; i++) 
    	{
			float y = ( 1f / (float) (numOfPoints-1) ) * (float) i;
        	float x = float.Parse(xOfPoints[i]);
        	linePoints[i].Set(x,y,0.0f);
    	}
		lineRenderer.positionCount = linePoints.Length;
    	lineRenderer.SetPositions(linePoints);
    }
}
