using UnityEngine;
using System.Collections;
using System.IO;

public class LinePoints : MonoBehaviour
{	
   static int numOfPoints = 160;
   static int numOfReferenceLines = 2321;
   float[] zOfPoints = {0.0000f, 0.0063f, 0.0126f, 0.0189f, 0.0252f,
   			       0.0314f, 0.0377f, 0.0440f, 0.0503f, 0.0566f, 
   			       0.0629f, 0.0692f, 0.0755f, 0.0818f, 0.0881f, 
   			       0.0943f, 0.1006f, 0.1069f, 0.1132f, 0.1195f, 
   			       0.1258f, 0.1321f, 0.1384f, 0.1447f, 0.1509f, 
   			       0.1572f, 0.1635f, 0.1698f, 0.1761f, 0.1824f, 
   			       0.1887f, 0.1950f, 0.2013f, 0.2075f, 0.2138f, 
   			       0.2201f, 0.2264f, 0.2327f, 0.2390f, 0.2453f, 
   			       0.2516f, 0.2579f, 0.2642f, 0.2704f, 0.2767f, 
   			       0.2830f, 0.2893f, 0.2956f, 0.3019f, 0.3082f, 
   			       0.3145f, 0.3208f, 0.3270f, 0.3333f, 0.3396f, 
   			       0.3459f, 0.3522f, 0.3585f, 0.3648f, 0.3711f, 
   			       0.3774f, 0.3836f, 0.3899f, 0.3962f, 0.4025f, 
   			       0.4088f, 0.4151f, 0.4214f, 0.4277f, 0.4340f, 
   			       0.4403f, 0.4465f, 0.4528f, 0.4591f, 0.4654f, 
   			       0.4717f, 0.4780f, 0.4843f, 0.4906f, 0.4969f, 
   			       0.5031f, 0.5094f, 0.5157f, 0.5220f, 0.5283f, 
   			       0.5346f, 0.5409f, 0.5472f, 0.5535f, 0.5597f, 
   			       0.5660f, 0.5723f, 0.5786f, 0.5849f, 0.5912f, 
   			       0.5975f, 0.6038f, 0.6101f, 0.6164f, 0.6226f, 
   			       0.6289f, 0.6352f, 0.6415f, 0.6478f, 0.6541f, 
   			       0.6604f, 0.6667f, 0.6730f, 0.6792f, 0.6855f, 
   			       0.6918f, 0.6981f, 0.7044f, 0.7107f, 0.7170f, 
   			       0.7233f, 0.7296f, 0.7358f, 0.7421f, 0.7484f, 
   			       0.7547f, 0.7610f, 0.7673f, 0.7736f, 0.7799f, 
   			       0.7862f, 0.7925f, 0.7987f, 0.8050f, 0.8113f, 
   			       0.8176f, 0.8239f, 0.8302f, 0.8365f, 0.8428f, 
   			       0.8491f, 0.8553f, 0.8616f, 0.8679f, 0.8742f, 
   			       0.8805f, 0.8868f, 0.8931f, 0.8994f, 0.9057f, 
   			       0.9119f, 0.9182f, 0.9245f, 0.9308f, 0.9371f, 
   			       0.9434f, 0.9497f, 0.9560f, 0.9623f, 0.9686f, 
   			       0.9748f, 0.9811f, 0.9874f, 0.9937f, 1.0000f};
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
    		float z = zOfPoints[i];
        	float x = float.Parse(xOfPoints[i]);
        	linePoints[i].Set(x,0.1f,z);
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
    		float z = zOfPoints[i];
        	float x = float.Parse(xOfPoints[i]);
        	linePoints[i].Set(x,0.1f,z);
    	}
		lineRenderer.positionCount = linePoints.Length;
    	lineRenderer.SetPositions(linePoints);
    }
}
