using Unity.Mathematics;
public static class GridHashHelper {
    public static int3 getPositionCellIndex(float3 position, float radius) {
        return new int3(math.floor(position/radius));
    }

    public static int hashCellIndex(int3 cellIndex) {
        int hash = cellIndex.x;
        hash = (hash * 397) ^ cellIndex.y;
        hash = (hash * 397) ^ cellIndex.z;
        hash += hash << 3;
        hash ^= hash >> 11;
        hash += hash << 15;
        return hash;
    }
}