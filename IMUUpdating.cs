using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System; 

public class IMUUpdating : MonoBehaviour
{
    public UDPReceive udpReceive;
    public string[] points;
    public float xcomp; 
    public float ycomp; 
    public float zcomp; 
    public Vector3 force; 
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
    }

    void FixedUpdate(){
        //Gathering UDPreceive data and updating the UI elements 
        string data = udpReceive.data;
        //UPDATES THE DATA AND GETS IN A PRESENTABLE FORM
        data = data.Remove(0, 1);
        data = data.Remove(data.Length-1, 1);
        points = data.Split(',');
        //data = data.Remove(0, 1);
        //data = data.Remove(data.Length-1, 1);
        //Add the values from the accelerometer and give it to force (reminder Z and Y in context are different here)
        force = new Vector3(float.Parse(points[3]),float.Parse(points[5]),float.Parse(points[4]));
        //If the thing is still then keep it still 
        if(Math.Abs(float.Parse(points[0])) < 0.008){
            xcomp = 0; 
        }
        else{
           xcomp = -1*float.Parse(points[0]);
        }
        if(Math.Abs(float.Parse(points[1])) < 0.008){
            ycomp = 0;
        }
        else{
            ycomp = -1*float.Parse(points[1]);
        }
        if(Math.Abs(float.Parse(points[2])) < 0.008){
            zcomp = 0;
        }
        else{
            zcomp = float.Parse(points[2]);
        }
        GetComponent<Rigidbody>().angularVelocity = new Vector3(xcomp,ycomp,zcomp);
        GetComponent<Rigidbody>().AddForce(force,ForceMode.Acceleration);
        //GetComponent<Rigidbody>().angularVelocity = new Vector3(0,0,float.Parse(points[2]));
        //GetComponent<Rigidbody>().angularVelocity = new Vector3(-1*float.Parse(points[0]),0,0);
        //GetComponent<Rigidbody>().angularVelocity = new Vector3(0,-1*float.Parse(points[1]),0);
        //GetComponent<Rigidbody>().angularVelocity = new Vector3(-1*float.Parse(points[0]),-1*float.Parse(points[1]),0);
    }
}
