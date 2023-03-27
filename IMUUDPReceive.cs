using UnityEngine;
using System; 
using System.Text; 
using System.Net; 
using System.Net.Sockets; 
using System.Threading; 
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
public class IMUUDPReceive : MonoBehaviour
{
    Thread receiveThread;
    UdpClient client; 
    public int port = 5052; 
    public bool startRecieving = true; 
    public bool printToConsole = false; 
    public string data; 
    public string[] points;
    public float testingdata; 
    // Start is called before the first frame update
    void Start()
    {
        client = new UdpClient(port);
        receiveThread = new Thread(
            new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true; 
        receiveThread.Start(); 
    }
    // receive thread 
    private void ReceiveData(){
    }
    // Update is called once per frame
    void Update()
    {
        try
        {
            IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
            byte[] dataByte = client.Receive(ref anyIP);
            data = Encoding.UTF8.GetString(dataByte);
            if (printToConsole){
                Debug.Log(data);
            }
            //UPDATES THE DATA AND GETS IN A PRESENTABLE FORM
            data = data.Remove(0, 1);
            data = data.Remove(data.Length-1, 1);
            //print(data);
            points = data.Split(',');
            /*for(int i = 1; i < points[0].Length; i++){
                points[i] = points[i].Remove(1, 1);
                points[i] = points[i].Remove(points[i].Length-1, 1);
            }*/
            testingdata = float.Parse(points[0]);
            //GETTING THE DATA TO THE 3D OBJECT USING QUATERNION 
            //Camera.main.transform.rotation = Quaternion.Euler(90,0,0);
            //transform.Rotate(float.Parse(points[0]), float.Parse(points[1]), float.Parse(points[2]), Space.Self);
        }
        catch (Exception err)
        {
            //print(err.ToString());
        }
        GetComponent<Rigidbody>().angularVelocity = new Vector3(-1*float.Parse(points[0]),-1*float.Parse(points[1]),-1*float.Parse(points[2]));
    }
}