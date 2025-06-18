using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class SonarTcpSubscriber : MonoBehaviour
{
    private string serverIp = "127.0.0.1";
    private int port = 7779;
    private TcpClient client;
    private NetworkStream stream;
    private Thread clientThread;
    private bool running = false;
    
    // Thread-safe transform data
    private Vector3 newPosition;
    private Vector3 newEulerAngles;
    private bool newDataAvailable = false;
    private readonly object dataLock = new object();
    private int connectionAttempts = 0;

    void Start() => StartConnectionThread();

    void Update()
    {
        if (newDataAvailable)
        {
            lock (dataLock)
            {
                transform.position = newPosition;
                transform.eulerAngles = newEulerAngles;
                newDataAvailable = false;
            }
        }
    }

    void StartConnectionThread()
    {
        if (clientThread != null && clientThread.IsAlive) return;
        
        running = true;
        clientThread = new Thread(RunClient);
        clientThread.IsBackground = true;
        clientThread.Start();
    }

    void RunClient()
    {
        while (running)
        {
            try
            {
                using (client = new TcpClient())
                {
                    Debug.Log($"Connecting to {serverIp}:{port}...");
                    client.Connect(serverIp, port);
                    stream = client.GetStream();
                    connectionAttempts = 0;
                    Debug.Log("Subscriber connected!");

                    byte[] buffer = new byte[4096];
                    StringBuilder dataBuffer = new StringBuilder();

                    while (running && client.Connected)
                    {
                        int bytesRead = stream.Read(buffer, 0, buffer.Length);
                        if (bytesRead == 0) // Graceful disconnect
                        {
                            Thread.Sleep(100);
                            continue;
                        }

                        string data = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                        dataBuffer.Append(data);

                        ProcessDataBuffer(dataBuffer);
                    }
                }
            }
            catch (SocketException) when (!running)
            {
                // Normal shutdown
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Subscriber error: {e.Message}");
            }
            finally
            {
                stream = null;
                client = null;
            }

            // Reconnection logic
            if (running)
            {
                connectionAttempts++;
                int delay = Mathf.Min(connectionAttempts * 2, 30); // Exponential backoff max 30s
                Debug.Log($"Reconnecting in {delay} seconds...");
                Thread.Sleep(delay * 1000);
            }
        }
    }

    void ProcessDataBuffer(StringBuilder dataBuffer)
    {
        while (dataBuffer.Length > 0 && dataBuffer.ToString().Contains("\n"))
        {
            int index = dataBuffer.ToString().IndexOf("\n");
            string line = dataBuffer.ToString(0, index).Trim();
            dataBuffer.Remove(0, index + 1);

            if (line.Length == 0) continue;

            string[] parts = line.Split(',');
            if (parts.Length != 6)
            {
                Debug.LogWarning($"Invalid data format. Expected 6 values, got {parts.Length}");
                continue;
            }

            if (TryParseVector3(parts, 0, out Vector3 position) &&
                TryParseVector3(parts, 3, out Vector3 rotation))
            {
                lock (dataLock)
                {
                    newPosition = position;
                    newEulerAngles = rotation;
                    newDataAvailable = true;
                }
            }
        }
    }

    bool TryParseVector3(string[] parts, int startIndex, out Vector3 result)
    {
        result = Vector3.zero;
        for (int i = 0; i < 3; i++)
        {
            if (!float.TryParse(parts[startIndex + i], out float val))
                return false;
            result[i] = val;
        }
        return true;
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