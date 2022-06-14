using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class BallPosition : MonoBehaviour
{	
	       
    void Start() 
    {
    	float ballPos = GameObject.Find("UDPComms").GetComponent<UDPReceive>().vivePos;
    	transform.position = new Vector3(ballPos, 0.2f, 0.05f);
    }
    
    void Update()
    {
    	float ballPos = GameObject.Find("UDPComms").GetComponent<UDPReceive>().vivePos;
    	transform.position = new Vector3(ballPos, 0.2f, 0.05f);
    }
}
