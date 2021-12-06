using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[GenerateAuthoringComponent]
public struct FluidProperties : ISharedComponentData
{
    public float radius;
    public float smoothingRadius;
    public float smoothingRadiusSq;

    public float mass;

    public float restDensity;
    public float viscosity;
    public float gravityMult;

    public float drag;
    
}
