using UnityEngine;

public class SonarRayCast : MonoBehaviour
{
    public bool showDebugRays = true;
    public int rayNumber = 10;
    public float view = 20f;
    public float rayLength = 50f;

    private float[] rayHits;
    private Transform vehicle;

    public float[] Hits => rayHits;

    void Start()
    {
        rayHits = new float[rayNumber];
        vehicle = this.transform;
    }

    void FixedUpdate()
    {
        DrawRays();
    }

    private void DrawRays()
    {
        float angleBetweenRays = view / (rayNumber - 1);
        float startAngle = -view / 2;

        for (int i = 0; i < rayNumber; i++)
        {
            float currentAngle = startAngle + angleBetweenRays * i;
            Vector3 direction = Quaternion.Euler(0, currentAngle, 0) * vehicle.forward;

            RaycastHit hit;
            if (Physics.Raycast(vehicle.position, direction, out hit, rayLength))
            {
                rayHits[i] = hit.distance;
                if (showDebugRays)
                {
                    Debug.DrawRay(vehicle.position, direction * hit.distance, Color.red);
                }
            }
            else
            {
                rayHits[i] = -1;
                if (showDebugRays)
                {
                    Debug.DrawRay(vehicle.position, direction * rayLength, Color.green);
                }
            }
        }
    }
}