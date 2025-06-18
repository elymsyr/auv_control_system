using UnityEngine;
using System.IO;

public class CameraPoser : MonoBehaviour
{
    private float offsetZ = 0.7f;
    public float rotationOffsetX = 90f;
    public Camera vehicleCamera;
    private Transform capsule;

    void Start()
    {
        capsule = transform;
    }

    void Update()
    {
        CameraPositioning();
    }

    void CameraPositioning()
    {
        if (capsule != null)
        {
            vehicleCamera.transform.position = capsule.position + capsule.forward * offsetZ;
            Quaternion capsuleRotation = Quaternion.LookRotation(capsule.forward);
            Quaternion rotationWithOffset = capsuleRotation * Quaternion.Euler(rotationOffsetX, 0, 0);
            vehicleCamera.transform.rotation = rotationWithOffset;
        }
    }
}