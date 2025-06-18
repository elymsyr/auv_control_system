using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class SonarTcpPublisher : MonoBehaviour
{
    public string serverIp = "127.0.0.1";
    public int serverPort = 7780;
    private SonarRayCast sonarRayCast;
    private TcpClient client;
    private NetworkStream stream;
    private Thread clientThread;
    private bool running = false;
    private int connectionAttempts = 0;

    void Start()
    {
        sonarRayCast = GetComponent<SonarRayCast>();
        running = true;
        clientThread = new Thread(ClientLoop);
        clientThread.IsBackground = true;
        clientThread.Start();
    }

    void ClientLoop()
    {
        while (running)
        {
            try
            {
                Debug.Log($"Connecting to {serverIp}:{serverPort}...");
                client = new TcpClient();
                client.Connect(serverIp, serverPort);
                stream = client.GetStream();
                connectionAttempts = 0;
                Debug.Log("Connected to sensor server.");

                while (running && client.Connected)
                {
                    if (sonarRayCast != null && sonarRayCast.Hits != null)
                    {
                        SendData(sonarRayCast.Hits);
                    }
                    Thread.Sleep(50);
                }
            }
            catch (SocketException) when (!running)
            {
                // Shutting down, ignore
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Connection error: {e.Message}");
            }
            finally
            {
                stream?.Close();
                client?.Close();
            }

            if (running)
            {
                connectionAttempts++;
                int delay = Mathf.Min(connectionAttempts * 2, 30);
                Debug.Log($"Reconnecting in {delay} seconds...");
                Thread.Sleep(delay * 1000);
            }
        }
    }

    void SendData(float[] hits)
    {
        try
        {
            var sb = new StringBuilder();
            for (int i = 0; i < hits.Length; i++)
            {
                sb.Append(hits[i].ToString("F4"));
                if (i < hits.Length - 1) sb.Append(',');
            }
            sb.Append('\n');

            byte[] data = Encoding.UTF8.GetBytes(sb.ToString());
            stream.Write(data, 0, data.Length);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Send error: {e.Message}");
            stream?.Close();
            client?.Close();
        }
    }

    void OnDestroy()
    {
        running = false;
        stream?.Close();
        client?.Close();
        if (clientThread != null && clientThread.IsAlive)
            clientThread.Join(500);
    }
}