using UnityEngine;
using System.Collections;

public class LinePoints : MonoBehaviour
{
    public Material LineMat;
    void Start() 
    {
    	int NumOfPoints = 1761;
    	// Read Reference.csv file
    	Vector3[] LinePoints = new Vector3[NumOfPoints];
    	var PointDataset = Resources.Load<TextAsset>("Reference");
    	var SplitDataset = PointDataset.text.Split('\n');
    	for (int i = 0; i < NumOfPoints; i++) 
    	{
    		var PointData = SplitDataset[i].Trim().Split(',');
    		float z = float.Parse(PointData[0]);
        	float x = float.Parse(PointData[1]);
        	LinePoints[i].Set(x-0.4f,0.1f,z);
    	}
    	// Create line
        LineRenderer RefLine = gameObject.AddComponent<LineRenderer>();
        RefLine.material = LineMat;
        RefLine.widthMultiplier = 0.05f;
        RefLine.positionCount = NumOfPoints;
        RefLine.SetPositions(LinePoints);
    }
}
