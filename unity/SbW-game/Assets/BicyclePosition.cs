using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class BicyclePosition : MonoBehaviour
{	
	       
    void Start() 
    {
    	float bicyclePos = GameObject.Find("UDPComms").GetComponent<UDPReceive>().vivePos;
    	transform.position = new Vector3(bicyclePos, 0.2f, 0.05f);
    }
    
    void Update()
    {
    	float bicyclePos = GameObject.Find("UDPComms").GetComponent<UDPReceive>().vivePos;
    	transform.position = new Vector3(bicyclePos, 0.2f, 0.05f);
    }
}
