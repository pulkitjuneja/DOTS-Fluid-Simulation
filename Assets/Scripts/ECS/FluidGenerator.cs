using Unity.Entities;
using UnityEngine;
using Unity.Collections;
using Unity.Transforms;
using Unity.Rendering;
using Unity.Mathematics;
public class FluidGenerator: MonoBehaviour
{

    public GameObject ParticlePrefab;
    public GameObject EnvColliderPrefab;
    private World DefaultWorld;
    EntityManager entityManager;

    public int count;

    void Start()
    {
        DefaultWorld = World.DefaultGameObjectInjectionWorld;
        entityManager = DefaultWorld.EntityManager;   
        GameObjectConversionSettings settings = GameObjectConversionSettings.FromWorld(DefaultWorld, null);
        Entity EntityPrefab = GameObjectConversionUtility.ConvertGameObjectHierarchy(ParticlePrefab, settings);

        NativeArray<Entity> entities = new NativeArray<Entity>(count, Allocator.Temp);
        entityManager.Instantiate(EntityPrefab, entities);

        for (int i = 0; i < count; i++)
        {
            entityManager.SetComponentData(entities[i], new Translation { Value = new float3(i % 16 + UnityEngine.Random.Range(-0.1f, 0.1f), 2 + (i / 16 / 16) * 1.1f, (i / 16) % 16) + UnityEngine.Random.Range(-0.1f, 0.1f) });
            entityManager.AddSharedComponentData<FluidProperties>(entities[i], new FluidProperties {
                radius = 1,
                smoothingRadius = 1,
                smoothingRadiusSq = 1,
                mass = 40,
                restDensity = 5000,
                viscosity = 0.1f,
                gravityMult = 1500,
                drag = 0.025f
            });
        }

        entities.Dispose();
        AddColliders();
    }

    private void AddColliders()
    {
        // Find all colliders
        GameObject[] colliders = GameObject.FindGameObjectsWithTag("EnvCollider");

        GameObjectConversionSettings settings = GameObjectConversionSettings.FromWorld(DefaultWorld, null);
        Entity ColliderPrefab = GameObjectConversionUtility.ConvertGameObjectHierarchy(EnvColliderPrefab, settings);

        // Turn them into entities
        NativeArray<Entity> entities = new NativeArray<Entity>(colliders.Length, Allocator.Temp);
        Debug.Log(entities.Length);
        entityManager.Instantiate(ColliderPrefab, entities);

        // Set data
        for (int i = 0; i < colliders.Length; i++)
        {
            entityManager.SetComponentData(entities[i], new EnvCollider
            {
                position = colliders[i].transform.position,
                right = colliders[i].transform.right,
                up = colliders[i].transform.up,
                scale = new float2(colliders[i].transform.localScale.x / 2f, colliders[i].transform.localScale.y / 2f)
            });
        }
    }
}
