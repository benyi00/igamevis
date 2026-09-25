#include "iGameFeatureEdgeRegionFilter.h"
#include <cmath>
#include <vector>
#include <unordered_set>
#include <utility>
#include <algorithm>
#include <map>

IGAME_NAMESPACE_BEGIN
namespace{
    constexpr double NORMAL_EPSILON = 1e-12;

    // 使用 Newell 方法计算面的单位法向量，支持三角形、四边形和一般多边形。
    // 法向方向由面顶点的绕序决定；非法面或退化面返回零向量。
    Vector3f getNormal(const SurfaceMesh::Pointer& mesh, igIndex faceId) {
        if (mesh == nullptr || faceId < 0 || faceId >= mesh->GetNumberOfFaces()) {
            return Vector3f(0.0f, 0.0f, 0.0f);
        }

        igIndex pointIds[IGAME_CELL_MAX_SIZE]{};
        const int numberOfPoints = mesh->GetFacePointIds(faceId, pointIds);
        if (numberOfPoints < 3) { return Vector3f(0.0f, 0.0f, 0.0f); }

        double nx = 0.0;
        double ny = 0.0;
        double nz = 0.0;

        for (int i = 0; i < numberOfPoints; ++i) {
            const Point& current = mesh->GetPoint(pointIds[i]);
            const Point& next = mesh->GetPoint(pointIds[(i + 1) % numberOfPoints]);

            nx += static_cast<double>(current[1] - next[1]) *
                    static_cast<double>(current[2] + next[2]);
            ny += static_cast<double>(current[2] - next[2]) *
                    static_cast<double>(current[0] + next[0]);
            nz += static_cast<double>(current[0] - next[0]) *
                    static_cast<double>(current[1] + next[1]);
        }

        const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (length <= NORMAL_EPSILON) { return Vector3f(0.0f, 0.0f, 0.0f); }

        return Vector3f(static_cast<float>(nx / length), static_cast<float>(ny / length),
                        static_cast<float>(nz / length));
    }

    class UnionFind {
    public:
        std::vector<int> parent;

        UnionFind(int numFaces) {
            parent = std::vector<int>(numFaces, -1);
            for (int faceId = 0; faceId < numFaces; faceId++) {
                parent[faceId] = faceId;
            } // Each face starts as its own parent.
        }

        void Union(int faceId1, int faceId2) {
            int root1 = FindParent(faceId1);
            int root2 = FindParent(faceId2);
            if (root1 != root2) { parent[root1] = root2; }
        }

        int FindParent(int id) {
            if (parent[id] != id) { parent[id] = FindParent(parent[id]); }
            return parent[id];
        }
    };
} // namespace


bool FeatureEdgeRegionFilter::Execute() {
	auto obj = GetInput(0);//Get the original mesh
    if (obj == nullptr) {
        std::cerr << "Failed to get input mesh" << std::endl;
        return false;
    }
    auto inputMesh = DynamicCast<SurfaceMesh>(obj);
    if (inputMesh == nullptr) {
        std::cerr << "Failed to get input surface mesh" << std::endl;
        return false;
    }
    auto mesh = SurfaceMesh::New();
    auto points = Points::New();
    points->DeepCopy(inputMesh->GetPoints());
    mesh->SetPoints(points);
    auto faces = CellArray::New();
    faces->GetOffset()->Reset(); // 清除构造时预置的 0
    if (!faces->DeepCopy(inputMesh->GetFaces())) { return false; }
    faces->DeepCopy(inputMesh->GetFaces());
    mesh->SetFaces(faces);
    auto attributeSet = AttributeSet::New();
    attributeSet->DeepCopy(inputMesh->GetAttributeSet());
    mesh->SetAttributeSet(attributeSet);
    /*mesh->SetPoints(inputMesh->GetPoints());
    mesh->SetFaces(inputMesh->GetFaces());
    mesh->SetAttributeSet(inputMesh->GetAttributeSet());*/

    mesh->BuildEdges();
    mesh->BuildEdgeLinks();
    mesh->BuildFaceEdgeLinks();
    mesh->BuildFaceLinks();

    const int numFaces = mesh->GetNumberOfFaces();
    UnionFind myUnion(numFaces);

    const int numEdges = mesh->GetNumberOfEdges();
    const int numPoints = mesh->GetNumberOfPoints();

    std::vector<Vector3f> normals{};
    for (int faceId = 0; faceId < numFaces; faceId++) normals.push_back(getNormal(mesh, faceId));

    for (int pointId = 0; pointId < numPoints; pointId++) {
        igIndex faceIds[IGAME_CELL_MAX_SIZE]{};
        int numNeightborFace = mesh->GetPointToNeighborFaces(pointId,faceIds);
        if (numNeightborFace <= 1) continue;
        for (int i = 0; i < numNeightborFace; i++) {//两两比较
            for (int j = i + 1; j < numNeightborFace; j++) {
                int id1 = faceIds[i];
                int id2 = faceIds[j];
                if (myUnion.FindParent(id1) == myUnion.FindParent(id2)) continue;//已经在同一个集合中了
                const auto normal1 = normals[id1];
                const auto normal2 = normals[id2];
                float dot = normal1.dot(normal2);
                if (dot > cos(m_featureAngle/180*M_PI)) {
                    myUnion.Union(id1, id2);
                }
            }
        }
    }

    //realign the region IDs
    std::map<IGint,IGint> regionIDs;
    auto regionArray = IntArray::New(); //save region ids
    regionArray->SetName("Region Id");
    regionArray->SetDimension(1);
    regionArray->Reserve(numFaces);
    int p = 0;
    for (int i = 0; i < numFaces; i++) { 
        int regionID = myUnion.FindParent(i);
        auto it = regionIDs.find(regionID);
        if (it == regionIDs.end()) {//this parent haven't be save in regionIDs
            regionIDs[regionID]=p;
            p++;
            if (p >= std::numeric_limits<IGint>::max()) {
                std::cerr << "Too many regions for IGint RegionId." << std::endl;
                return false;
            }
        }
        regionArray->AddValue(regionIDs[regionID]);
    }

    //auto attributeSet = mesh->GetAttributeSet();
    if (!attributeSet) { return false; }
    auto oldAttributeId = attributeSet->GetAttributeIndex("Region Id");
    if (oldAttributeId >= 0) { 
        auto oldArray = DynamicCast<IntArray>(attributeSet->GetAttribute("Region Id").pointer);
        if (oldArray == nullptr) {
            std::cerr << "Region Id already exists, but it is not an IntArray." << std::endl;
            return false;
        }
        oldArray->SetDimension(1);
        oldArray->Resize(numFaces);
        for (int i = 0; i < numFaces; ++i) { oldArray->SetValue(i, regionArray->GetValue(i)); }
        oldArray->Modified();
        attributeSet->GetAttribute("Region Id").UpdateAllDataRange();
    } else {
        attributeSet->AddScalar(IG_CELL, regionArray);
    }
    attributeSet->ForceReConvertToDrawableData();
    mesh->SetName(inputMesh->GetName() + "_regionId");

    std::map<int, int> regionFaceCount;
    for (int i = 0; i < numFaces; i++) {
        int root = myUnion.FindParent(i);
        int regionId = regionIDs[root];
        regionFaceCount[regionId]++;
    }
    //for (auto& item: regionFaceCount) {//print id and faces number of every region
    //    std::cout << "Region " << item.first << " faces: " << item.second << std::endl;
    //}
    std::cout << "Number of regions:" << p<<std::endl;
    SetOutput(mesh);
    return true;

}
IGAME_NAMESPACE_END
