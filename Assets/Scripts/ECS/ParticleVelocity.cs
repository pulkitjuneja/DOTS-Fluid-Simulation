using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[GenerateAuthoringComponent]
public struct ParticleVelocity : IComponentData
{
    public float3 Value;
}
