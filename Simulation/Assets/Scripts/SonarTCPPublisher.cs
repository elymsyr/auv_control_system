using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class SonarTcpPublisher : MonoBehaviour
{
    private string serverIp = "127.0.0.1";
    private int serverPort = 7780;
    private SonarRayCast sonarRayCast;
    private TcpClient client;
    private NetworkStream stream;
    private Thread clientThread;
    private bool running = false;
    private int connectionAttempts = 0;
    private readonly object sendLock = new object();

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
                    Thread.Sleep(50); // 20Hz update rate
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
            // Build data string
            var sb = new StringBuilder();
            for (int i = 0; i < hits.Length; i++)
            {
                sb.Append(hits[i].ToString("F4"));
                if (i < hits.Length - 1) sb.Append(',');
            }

            // Convert to bytes
            byte[] dataBytes = Encoding.UTF8.GetBytes(sb.ToString());
            
            // Create header with message length (big-endian)
            byte[] lengthHeader = new byte[4];
            int length = dataBytes.Length;
            lengthHeader[0] = (byte)(length >> 24);
            lengthHeader[1] = (byte)(length >> 16);
            lengthHeader[2] = (byte)(length >> 8);
            lengthHeader[3] = (byte)length;
            
            // Send with lock to prevent interleaving
            lock (sendLock)
            {
                stream.Write(lengthHeader, 0, 4);
                stream.Write(dataBytes, 0, dataBytes.Length);
            }
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