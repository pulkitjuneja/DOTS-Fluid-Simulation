using Unity.Burst;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine;
using Unity.Mathematics;
using Unity.Transforms;

[UpdateBefore(typeof(TransformSystemGroup))]
public class FluidSimulation : SystemBase
{
    private EntityQuery ParticlesQuery;
    private EntityQuery ColliderQuery;
    private JobHandle collidersToNativeArrayJobHandle;
    private NativeArray<EnvCollider> colliders;
    private List<FluidProperties> fluidProperties = new List<FluidProperties>();

    ParticleData disposableParticleData;

    public readonly static int3[] cellOffsets =
    {
        new int3(1, 1, 1), new int3(1, 1, 0), new int3(1, 1, -1), new int3(1, 0, 1),
        new int3(1, 0, 0), new int3(1, 0, -1),new int3(1, -1, 1), new int3(1, -1, 0),
        new int3(1, -1, -1), new int3(0, 1, 1), new int3(0, 1, 0), new int3(0, 1, -1),
        new int3(0, 0, 1), new int3(0, 0, 0), new int3(0, 0, -1), new int3(0, -1, 1),
        new int3(0, -1, 0), new int3(0, -1, -1), new int3(-1, 1, 1), new int3(-1, 1, 0),
        new int3(-1, 1, -1), new int3(-1, 0, 1), new int3(-1, 0, 0), new int3(-1, 0, -1),
        new int3(-1, -1, 1), new int3(-1, -1, 0), new int3(-1, -1, -1)
    };

    struct ParticleData {
        public NativeArray<Translation> particlesPosition;
        public NativeArray<ParticleVelocity> particlesVelocity;
        public NativeArray<float3> particlesForces;
        public NativeArray<float> particlesPressure;
        public NativeArray<float> particlesDensity;
    }

    private struct HashPositions : IJobParallelFor
    {
        #pragma warning disable 0649
        [ReadOnly] public float cellRadius;

        public NativeArray<Translation> positions;
        public NativeMultiHashMap<int, int>.ParallelWriter hashMap;
        #pragma warning restore 0649

        public void Execute(int index)
        {
            float3 position = positions[index].Value;

            int3 cellIndex = GridHashHelper.getPositionCellIndex(position, cellRadius);
            int hash = GridHashHelper.hashCellIndex(cellIndex);
            hashMap.Add(hash, index);

            positions[index] = new Translation { Value = position };
        }
    }

    private struct ComputeDensityPressure : IJobParallelFor
    {
        #pragma warning disable 0649
        [ReadOnly] public NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<Translation> particlesPosition;
        [ReadOnly] public FluidProperties properties;

        public NativeArray<float> densities;
        public NativeArray<float> pressures;
        #pragma warning restore 0649

        private const float PI = 3.14159274F;
        private const float GAS_CONST = 2000.0f;

        public void Execute(int index)
        {
            float3 position = particlesPosition[index].Value;
            float density = 0.0f;
            int3 gridPosition = new int3(math.floor(position/properties.radius));
            bool found;

            for(int i = 0; i< 27; i++) {
                int3 gridOffset = cellOffsets[i];
                int hash = GridHashHelper.hashCellIndex(gridPosition + gridOffset);
                NativeMultiHashMapIterator<int> iterator;
                int other;
                found = hashMap.TryGetFirstValue(hash, out other, out iterator);
                while(found) {
                    float3 rij = particlesPosition[other].Value - position;
                    float r2 = math.lengthsq(rij);
                    if (r2 < properties.smoothingRadiusSq) {
                        density += properties.mass * (315.0f / (64.0f * PI * math.pow(properties.smoothingRadius, 9.0f))) * math.pow(properties.smoothingRadiusSq - r2, 3.0f);
                    }
                    found = hashMap.TryGetNextValue(out other, ref iterator);
                }
            }

            densities[index] = density;
            pressures[index] = GAS_CONST * (density - properties.restDensity);    
        }
    }


    private struct ComputeForces : IJobParallelFor
    {
        #pragma warning disable 0649
        [ReadOnly] public NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<Translation> particlesPosition;
        [ReadOnly] public NativeArray<ParticleVelocity> particlesVelocity;
        [ReadOnly] public NativeArray<float> particlesPressure;
        [ReadOnly] public NativeArray<float> particlesDensity;
        [ReadOnly] public FluidProperties properties;

        public NativeArray<float3> particlesForces;
        #pragma warning restore 0649

        private const float PI = 3.14159274F;



        public void Execute(int index)
        {
            // Cache
            int particleCount = particlesPosition.Length;
            float3 position = particlesPosition[index].Value;
            float3 velocity = particlesVelocity[index].Value;
            float pressure = particlesPressure[index];
            float density = particlesDensity[index];
            float3 forcePressure = new float3(0, 0, 0);
            float3 forceViscosity = new float3(0, 0, 0);
            int3 gridPosition = GridHashHelper.getPositionCellIndex(position, properties.radius);
            bool found;

            for(int i = 0; i< 27; i++) {
                int3 gridOffset = cellOffsets[i];
                int hash = GridHashHelper.hashCellIndex(gridPosition + gridOffset);
                NativeMultiHashMapIterator<int> iterator;
                int other;
                found = hashMap.TryGetFirstValue(hash, out other, out iterator);
                while(found) {
                    // same particle
                    if(index == other) {
                        found = hashMap.TryGetNextValue(out other, ref iterator);
                        continue;       
                    }

                    float3 rij = particlesPosition[other].Value - position;
                    float r2 = math.lengthsq(rij);
                    float r = math.sqrt(r2);
                    if (r < properties.smoothingRadius) {
                        forcePressure += -math.normalize(rij) * properties.mass * (2.0f * pressure) / (2.0f * density) * (-45.0f / (PI * math.pow(properties.smoothingRadius, 6.0f))) * math.pow(properties.smoothingRadius - r, 2.0f);
                        forceViscosity += properties.viscosity * properties.mass * (particlesVelocity[other].Value - velocity) / density * (45.0f / (PI * math.pow(properties.smoothingRadius, 6.0f))) * (properties.smoothingRadius - r);
                    }
                    found = hashMap.TryGetNextValue(out other, ref iterator);
                }
            }

            float3 forceGravity = new float3(0.0f, -9.81f, 0.0f) * density * properties.gravityMult;
            particlesForces[index] = forcePressure + forceViscosity + forceGravity;
        }
    }

     private struct ComputeColliders : IJobParallelFor
    {
        #pragma warning disable 0649
        [ReadOnly] public FluidProperties properties;
        [ReadOnly] public NativeArray<EnvCollider> copyColliders;

        public NativeArray<Translation> particlesPosition;
        public NativeArray<ParticleVelocity> particlesVelocity;
        #pragma warning restore 0649

        private const float BOUND_DAMPING = -0.5f;



        private static bool Intersect(EnvCollider collider, float3 position, float radius, out float3 penetrationNormal, out float3 penetrationPosition, out float penetrationLength)
        {
            float3 colliderProjection = collider.position - position;

            penetrationNormal = math.cross(collider.right, collider.up);
            penetrationLength = math.abs(math.dot(colliderProjection, penetrationNormal)) - (radius / 2.0f);
            penetrationPosition = collider.position - colliderProjection;

            return penetrationLength < 0.0f
                && math.abs(math.dot(colliderProjection, collider.right)) < collider.scale.x
                && math.abs(math.dot(colliderProjection, collider.up)) < collider.scale.y;
        }



        private static Vector3 DampVelocity(EnvCollider collider, float3 velocity, float3 penetrationNormal, float drag)
        {
            float3 newVelocity = math.dot(velocity, penetrationNormal) * penetrationNormal * BOUND_DAMPING
                                + math.dot(velocity, collider.right) * collider.right * drag
                                + math.dot(velocity, collider.up) * collider.up * drag;
            newVelocity = math.dot(newVelocity, new float3(0, 0, 1)) * new float3(0, 0, 1)
                        + math.dot(newVelocity, new float3(1, 0, 0)) * new float3(1, 0, 0)
                        + math.dot(newVelocity, new float3(0, 1, 0)) * new float3(0, 1, 0);
            return newVelocity;
        }



        public void Execute(int index)
        {
            // Cache
            int colliderCount = copyColliders.Length;
            float3 position = particlesPosition[index].Value;
            float3 velocity = particlesVelocity[index].Value;

            // Process
            for (int i = 0; i < colliderCount; i++)
            {
                float3 penetrationNormal;
                float3 penetrationPosition;
                float penetrationLength;
                if (Intersect(copyColliders[i], position, properties.radius, out penetrationNormal, out penetrationPosition, out penetrationLength))
                {
                    velocity = DampVelocity(copyColliders[i], velocity, penetrationNormal, 1.0f - properties.drag);
                    position = penetrationPosition - penetrationNormal * math.abs(penetrationLength);
                }
            }

            // Apply
            particlesVelocity[index] = new ParticleVelocity { Value = velocity };
            particlesPosition[index] = new Translation { Value = position };
        }
    }


    public struct ApplyForces : IJobParallelFor
    {
        #pragma warning disable 0649

        [ReadOnly] public NativeArray<float3> particlesForces;
        [ReadOnly] public NativeArray<float> particlesDensity;
        public NativeArray<Translation> particlesPosition;
        public NativeArray<ParticleVelocity> particlesVelocity;

        public void Execute(int index)
        {
            float3 velocity = particlesVelocity[index].Value;
            float3 position = particlesPosition[index].Value;

            // Debug.Log(particlesDensity[index]);

            velocity += 0.0008f * particlesForces[index] / particlesDensity[index];
            position += 0.0008f * velocity;

            particlesVelocity[index] = new ParticleVelocity { Value = velocity };
            particlesPosition[index] = new Translation { Value = position };
        }
    }

    protected override void OnCreate()
    {
        ParticlesQuery = GetEntityQuery(ComponentType.ReadOnly(typeof(FluidProperties)), typeof(Translation), typeof(ParticleVelocity));
        ColliderQuery = GetEntityQuery(ComponentType.ReadOnly(typeof(EnvCollider)));
    }

    protected override void OnStartRunning()
    {
        colliders = ColliderQuery.ToComponentDataArray<EnvCollider>(Allocator.Persistent);
    }
    
    protected override void OnUpdate()
    {
        NativeArray<Translation> particlesPosition = ParticlesQuery.ToComponentDataArray<Translation>(Allocator.TempJob);
        NativeArray<ParticleVelocity> particlesVelocity = ParticlesQuery.ToComponentDataArray<ParticleVelocity>(Allocator.TempJob);
        EntityManager.GetAllUniqueSharedComponentData(fluidProperties);

        int particleCount = particlesPosition.Length;
        NativeMultiHashMap<int, int> hashMap = new NativeMultiHashMap<int, int>(particleCount, Allocator.TempJob);

        NativeArray<float3> particlesForces = new NativeArray<float3>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        NativeArray<float> particlesPressure = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        NativeArray<float> particlesDensity = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
    
        disposableParticleData = new ParticleData {
            particlesPosition = particlesPosition,
            particlesVelocity = particlesVelocity,
            particlesForces = particlesForces,
            particlesPressure = particlesPressure,
            particlesDensity = particlesDensity
        };

        // initialize native arrays with default position
        MemsetNativeArray<float> pressuresInitJob = new MemsetNativeArray<float> { Source = particlesPressure, Value = -1.0f };
        JobHandle pressureInitJobHandle = pressuresInitJob.Schedule(particleCount, 64);
        MemsetNativeArray<float> densityInitJob = new MemsetNativeArray<float> { Source = particlesDensity, Value = -1.0f };
        JobHandle densityInitJobHandle = densityInitJob.Schedule(particleCount, 64);
        MemsetNativeArray<float3> forcesInitJob = new MemsetNativeArray<float3> { Source = particlesForces, Value = new float3(-1, -1, -1) };
        JobHandle forceInitJobHandle = forcesInitJob.Schedule(particleCount, 64);


        HashPositions hashPositionsJob = new HashPositions
        {
            positions = particlesPosition,
            hashMap = hashMap.AsParallelWriter(),
            cellRadius = fluidProperties[1].radius
        };

        JobHandle gridHashJob = hashPositionsJob.Schedule(particleCount, 64);   

        JobHandle computePressureJobDeps = JobHandle.CombineDependencies(gridHashJob,
        pressureInitJobHandle, densityInitJobHandle);

        ComputeDensityPressure computeDensityPressureJob = new ComputeDensityPressure
        {
            particlesPosition = particlesPosition,
            densities = particlesDensity,
            pressures = particlesPressure,
            hashMap = hashMap,
            properties = fluidProperties[1]
        };

        JobHandle computeDensPressJobHandle = computeDensityPressureJob.Schedule(particleCount, 64, computePressureJobDeps);

        JobHandle ComputeForceJobDeps = JobHandle.CombineDependencies(computeDensPressJobHandle, forceInitJobHandle);

        ComputeForces computeForcesJob = new ComputeForces
        {
            particlesPosition = particlesPosition,
            particlesVelocity = particlesVelocity,
            particlesForces = particlesForces,
            particlesPressure = particlesPressure,
            particlesDensity = particlesDensity,
            hashMap = hashMap,
            properties = fluidProperties[1]
        };

        JobHandle ComputeForcesJobHandle = computeForcesJob.Schedule(particleCount, 64, ComputeForceJobDeps);

        ApplyForces applyForcesJob = new ApplyForces {
            particlesPosition = particlesPosition,
            particlesVelocity = particlesVelocity,
            particlesDensity = particlesDensity,
            particlesForces = particlesForces
        };

        JobHandle applyForcesJobHandle = applyForcesJob.Schedule(particleCount, 64, ComputeForcesJobHandle);

        ComputeColliders computeCollidersJob = new ComputeColliders
        {
            particlesPosition = particlesPosition,
            particlesVelocity = particlesVelocity,
            copyColliders = colliders,
            properties = fluidProperties[1]
        };

        JobHandle computeCollidersJobHandle = computeCollidersJob.Schedule(particleCount, 64, applyForcesJobHandle);

        JobHandle finalHandle = Entities.ForEach((int entityInQueryIndex, ref Translation t, ref ParticleVelocity velocity)=> {
            t.Value = particlesPosition[entityInQueryIndex].Value;
            velocity.Value = particlesVelocity[entityInQueryIndex].Value;
        }).Schedule(computeCollidersJobHandle);

        finalHandle.Complete();

        // Dispose of particle data
        disposableParticleData.particlesPosition.Dispose();
        disposableParticleData.particlesVelocity.Dispose();
        disposableParticleData.particlesForces.Dispose();
        disposableParticleData.particlesPressure.Dispose();
        disposableParticleData.particlesDensity.Dispose();
        hashMap.Dispose();
    }

    // protected override void OnStopRunning()
    // {
    //     EntityManager.CompleteAllJobs();
    //     disposableParticleData.particlesPosition.Dispose();
    //     disposableParticleData.particlesVelocity.Dispose();
    // }
}
