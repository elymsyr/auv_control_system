using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class EnvironmentTcpSubscriber : MonoBehaviour
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

                    byte[] lengthBuffer = new byte[4];
                    byte[] dataBuffer = null;
                    int bytesRead = 0;
                    int bytesToRead = 0;

                    while (running && client.Connected)
                    {
                        // Read 4-byte message length header
                        bytesRead = 0;
                        while (bytesRead < 4 && running)
                        {
                            int read = stream.Read(
                                lengthBuffer, 
                                bytesRead, 
                                4 - bytesRead
                            );
                            if (read == 0) break; // Disconnected
                            bytesRead += read;
                        }

                        if (bytesRead < 4) continue; // Incomplete header

                        // Get message length
                        bytesToRead = BitConverter.ToInt32(lengthBuffer, 0);
                        if (bytesToRead <= 0 || bytesToRead > 1024 * 1024) // Sanity check
                        {
                            Debug.LogError($"Invalid message length: {bytesToRead}");
                            break;
                        }

                        // Read message data
                        dataBuffer = new byte[bytesToRead];
                        bytesRead = 0;
                        while (bytesRead < bytesToRead && running)
                        {
                            int read = stream.Read(
                                dataBuffer, 
                                bytesRead, 
                                bytesToRead - bytesRead
                            );
                            if (read == 0) break; // Disconnected
                            bytesRead += read;
                        }

                        if (bytesRead < bytesToRead) continue; // Incomplete message

                        // Process complete message
                        string message = Encoding.UTF8.GetString(dataBuffer);
                        ProcessMessage(message);
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

    void ProcessMessage(string message)
    {
        string[] parts = message.Split(',');
        if (parts.Length != 6)
        {
            Debug.LogWarning($"Invalid data format. Expected 6 values, got {parts.Length}");
            return;
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

    bool TryParseVector3(string[] parts, int startIndex, out Vector3 result)
    {
        result = Vector3.zero;
        for (int i = 0; i < 3; i++)
        {
            if (!float.TryParse(parts[startIndex + i], out float val))
            {
                Debug.LogWarning($"Failed to parse float: {parts[startIndex + i]}");
                return false;
            }
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