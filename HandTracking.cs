using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System; 
 
public class HandTracking : MonoBehaviour
{
    // Start is called before the first frame update
    public UDPReceive udpReceive;
    public GameObject[] handPoints;
    public Vector3 handAngle; 
    public float handAngleX; 
    public float handAngleY;
    public float handAngleZ; 
    public Vector3 vector1;
    public Vector3 vector2; 
    void Start()
    {
        
    }
 
    // Update is called once per frame
    void Update()
    {
        string data = udpReceive.data;
        data = data.Remove(0, 1);
        data = data.Remove(data.Length-1, 1);
        //print(data);
        string[] points = data.Split(',');
        //print(points[0]);
        // 
        //0        1*3      2*3
        //x1,y1,z1,x2,y2,z2,x3,y3,z3
        //Getting the Hand Angle using vector calculus, arctan(points[0].transform.y,points[0].transform.z))
        //rotation 
        //thumb: 
        /*Vector3 wrist = new Vector3(points[0],points[1],points[2]);
        Vector3 thumb1 = new Vector3(points[3],points[4],points[5]);
        float thumb1angle = Math.Acos();
        Vector3 thumb2 = new Vector3(points[6],points[7],points[8]);*/
        //handAngleX = Math.Acos(Vector3.Dot(points[0].transform.localPosition))
        //handAngle = new Vector3(Math.Acos(Vector3.Dot(points[0].transform,points[9].transform)));
        for ( int i = 0; i<21; i++)
        {
            float x = (float.Parse(points[i * 3]))*10;
            float y = (-1*float.Parse(points[i * 3 + 1]))*10;
            float z = (float.Parse(points[i * 3 + 2]))*10;
 
            handPoints[i].transform.localPosition = new Vector3(6*-1*x,6*y, 6*z);
 
        }
        /*GameObject.Find("Hand").transform.eulerAngles = new Vector3((float)Math.Atan(handPoints[9].transform.localPosition.y/handPoints[9].transform.localPosition.z),
        (float)Math.Atan(handPoints[9].transform.localPosition.z/handPoints[9].transform.localPosition.x),
        (float)Math.Atan(handPoints[9].transform.localPosition.y/handPoints[9].transform.localPosition.x));*/
        // Depth sensor can't really do close ups :(
        GameObject.Find("DemoHand").transform.localPosition = new Vector3(float.Parse(points[63])*30,float.Parse(points[64])*30,float.Parse(points[65]));
        //GameObject.Find("DemoHand").transform.localPosition = new Vector3(float.Parse(points[63]) + 12.672f,float.Parse(points[64]),float.Parse(points[65]) + 0.964f);
    }
}
