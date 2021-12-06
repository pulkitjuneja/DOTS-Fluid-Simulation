using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[GenerateAuthoringComponent]
public struct EnvCollider : IComponentData
{
    public float3 position;
    public float3 right;
    public float3 up;
    public float2 scale;
    
}
