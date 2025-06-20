using UnityEngine;
using System.Collections.Generic;

public class AsteroidFieldGenerator : MonoBehaviour
{
    [Header("Field Settings")]
    public int asteroidCount = 200;
    public Vector3 fieldSize = new Vector3(100, 50, 100);
    public float safeZoneRadius = 20f;

    [Header("Asteroid Settings")]
    public float minScale = 1f;
    public float maxScale = 8f;
    public float minRotationSpeed = 5f;
    public float maxRotationSpeed = 30f;
    public Material[] asteroidMaterials;

    private List<GameObject> asteroids = new List<GameObject>();

    void Start()
    {
        GenerateAsteroidField();
    }

    void GenerateAsteroidField()
    {
        // Clear existing asteroids if regenerating
        ClearAsteroids();

        for (int i = 0; i < asteroidCount; i++)
        {
            Vector3 position;
            int attempts = 0;
            bool positionValid = false;

            // Find a valid position that's not in the safe zone
            do
            {
                position = new Vector3(
                    Random.Range(-fieldSize.x / 2, fieldSize.x / 2),
                    Random.Range(-fieldSize.y / 2, fieldSize.y / 2),
                    Random.Range(-fieldSize.z / 2, fieldSize.z / 2)
                );

                // Ensure we're not too close to the origin
                positionValid = position.magnitude > safeZoneRadius;
                attempts++;
            } 
            while (!positionValid && attempts < 100);

            if (positionValid)
            {
                CreateAsteroid(position);
            }
        }

        Debug.Log($"Created {asteroids.Count} asteroids in the field");
    }

    void CreateAsteroid(Vector3 position)
    {
        // Create asteroid and set basic properties
        GameObject asteroid = GameObject.CreatePrimitive(PrimitiveType.Cube);
        asteroid.name = "Asteroid";
        asteroid.transform.position = position;
        asteroid.transform.SetParent(transform);
        
        // Random scale with some irregularity
        Vector3 baseScale = Vector3.one * Random.Range(minScale, maxScale);
        asteroid.transform.localScale = new Vector3(
            baseScale.x * Random.Range(0.8f, 1.2f),
            baseScale.y * Random.Range(0.7f, 1.3f),
            baseScale.z * Random.Range(0.9f, 1.1f)
        );
        
        // Add random rotation
        asteroid.transform.rotation = Random.rotation;
        
        // Add rotation component
        RotatingAsteroid rotator = asteroid.AddComponent<RotatingAsteroid>();
        rotator.rotationSpeed = new Vector3(
            Random.Range(-maxRotationSpeed, maxRotationSpeed),
            Random.Range(-maxRotationSpeed, maxRotationSpeed),
            Random.Range(-maxRotationSpeed, maxRotationSpeed)
        );
        
        // Apply random material
        if (asteroidMaterials != null && asteroidMaterials.Length > 0)
        {
            Renderer renderer = asteroid.GetComponent<Renderer>();
            renderer.material = asteroidMaterials[Random.Range(0, asteroidMaterials.Length)];
        }
        
        // Add collider if not already present
        if (!asteroid.GetComponent<Collider>())
        {
            asteroid.AddComponent<BoxCollider>();
        }

        asteroids.Add(asteroid);
    }

    void ClearAsteroids()
    {
        foreach (GameObject asteroid in asteroids)
        {
            if (asteroid != null)
            {
                Destroy(asteroid);
            }
        }
        asteroids.Clear();
    }

    // Editor button for regeneration
    #if UNITY_EDITOR
    [ContextMenu("Regenerate Asteroid Field")]
    void RegenerateField()
    {
        GenerateAsteroidField();
    }
    #endif

    // Visualize the field area in the editor
    void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(1, 0.5f, 0, 0.3f);
        Gizmos.DrawWireCube(transform.position, fieldSize);
        
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(Vector3.zero, safeZoneRadius);
    }
}

// Separate component for asteroid rotation
public class RotatingAsteroid : MonoBehaviour
{
    public Vector3 rotationSpeed;
    
    void Update()
    {
        transform.Rotate(rotationSpeed * Time.deltaTime);
    }
}