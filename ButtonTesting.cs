using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ButtonTesting : MonoBehaviour
{
    public UDPReceive udpReceive;
    public float wallDistance; 
    public Vector3 wallCoord1;
    public Vector3 wallCoord2;
    public Vector3 wallCoord3;
    public string[] points; 
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        //Gathering UDPreceive data and updating the UI elements 
        string data = udpReceive.data;
        //UPDATES THE DATA AND GETS IN A PRESENTABLE FORM
        data = data.Remove(0, 1);
        data = data.Remove(data.Length-1, 1);
        points = data.Split(',');
        //data = data.Remove(0, 1);
        //data = data.Remove(data.Length-1, 1);
        wallDistance = float.Parse(points[6]);
        GameObject.Find("WallDistanceMeter").GetComponent<TMPro.TextMeshProUGUI>().text = points[6]; 
    }

    public static void DebugButton(){
        Debug.Log("Pressed");
    }

    public static void PlaceWall(){
        Debug.Log("Wall Placed");
    }
}
