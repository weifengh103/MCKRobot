using System;
using System.Net;
using System.Runtime.Remoting.Messaging;

using uPLibrary.Networking.M2Mqtt.Messages;
using uPLibrary.Networking.M2Mqtt;

public class RobotMqttClient
{
    public MqttClient client;
    public void Init()
    {

        // create client instance 
        MqttClient client = new MqttClient(IPAddress.Parse("127.0.0.1"));

        // register to message received 
        //client.MqttMsgPublishReceived += client_MqttMsgPublishReceived;

        string clientId = Guid.NewGuid().ToString();
        client.Connect(clientId);

        // subscribe to the topic "/home/temperature" with QoS 2 
        client.Subscribe(new string[] { "topic2" }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
        client.Subscribe(new string[] { "topic1" }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });

    }

    public void Subscribe(string topic)
    {
        client.Subscribe(new string[] { topic }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
        //client.Subscribe(new string[] { "topic1" }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
    }


    static void client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e)
        {
        // handle message received 
        var str = System.Text.Encoding.Default.GetString(e.Message);

        Console.WriteLine(str);
        }
    

    
}
