#include "iGameQEM.h"
#include <Eigen/eigen>
#include <queue>
#include <vector>

IGAME_NAMESPACE_BEGIN
namespace
{
struct computedEdge {
    int edgeId;
    float distance;
    Vector3f position;
    int version;

    computedEdge(int id, float d, Vector3f p,int ver) {
        edgeId = id;
        distance = d;
        position = p;
        version = ver;
    }
    
};

struct Compare {
    bool operator()(const computedEdge& A,const computedEdge &B) {
        return A.distance > B.distance;
    }
};

Eigen::Matrix4f ComputeQ(Face* face) { 
    auto iGnormal = face->GetNormal();
    auto toEigen = [](const Vector<float, 3>& p) -> Eigen::Vector3d {
        return {double(p[0]), double(p[1]), double(p[2])};
    };
    auto normal = toEigen(iGnormal);
    float a = normal.x();
    float b = normal.y();
    float c = normal.z();
    Point v1 = face->GetPoint(0);
    float d = -(a * v1[0] + b * v1[1] + c * v1[2]);

    Eigen::Matrix4f m;
    m<< a*a,a*b,a*c,a*d,
        a*b,b*b,b*c,b*d,
        a*c,b*c,c*c,c*d,
        a*d,b*d,c*d,d*d;
    return m;
}

Eigen::Vector3f ComputeV(Eigen::Matrix4f Q){ 
    Eigen::Matrix3f A;
    A << Q(0, 0), Q(0, 1), Q(0, 2),
        Q(1, 0), Q(1, 1), Q(1, 2),
        Q(2, 0), Q(2, 1), Q(2, 2);
    Eigen::Vector3f b;
    b << Q(0, 3), Q(1, 3), Q(2, 3);
    Eigen::Vector3f v = A.completeOrthogonalDecomposition().solve(-b);
    return v;
}

float ComputeQuadricError(Eigen::Matrix4f Q, Eigen::Vector3f v) {
    Eigen::Vector4f v4;
    v4 << v, 1.0f;
    auto distance = v4.transpose() * Q * v4;
    return distance.value();
}

void ComputePointQ(SurfaceMesh::Pointer mesh, std::vector<Eigen::Matrix4f> &Quadrics) {//把每个点的Q都算一次，放进vector里
    int numPoints = mesh->GetNumberOfPoints();
    for (int point = 0; point < numPoints; point++) {
        Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
        int faces[IGAME_CELL_MAX_SIZE];
        int numFaces = mesh->GetPointToNeighborFaces(point, faces);
        for (int i = 0; i < numFaces; i++) {
            auto face = mesh->GetFace(faces[i]);
            Q += ComputeQ(face);
        }
        Quadrics[point] = Q;
    }
}

computedEdge ComputeEdge(int edgeId, SurfaceMesh::Pointer mesh, std::vector<int>& version,
                         const std::vector<Eigen::Matrix4f>& Quadrics) {
    Line* edge = mesh->GetEdge(edgeId);
    Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
    for (int i = 0; i < 2; i++) {
        int point = edge->GetPointId(i);
        Q += Quadrics[point];
    }
    Eigen::Vector3f v = ComputeV(Q);
    float error = ComputeQuadricError(Q, v);
    Vector3f igameV(v[0], v[1], v[2]);
    computedEdge node(edgeId, error, igameV, version[edgeId]+1);
    version[edgeId]++;
    return node;
}

void UpdateEdges(std::priority_queue<computedEdge, std::vector<computedEdge>,Compare>& edgeQueue, int pointId,
                 SurfaceMesh::Pointer mesh,std::vector<Eigen::Matrix4f> &Quadrics,
                 std::vector<int>& version) {//边坍缩成点后，它周围的边要重新计算Q和error
    int edgeIds[IGAME_CELL_MAX_SIZE];
    int numEdges = mesh->GetPointToNeighborEdges(pointId, edgeIds);
    for (int i = 0; i < numEdges; i++) { 
        edgeQueue.push(ComputeEdge(edgeIds[i],mesh,version,Quadrics));
    }
}

}

bool QEMFilter::Execute() {
    auto obj = GetInput(0);
    if (obj == nullptr) return false;
    if (obj->GetDataObjectType() != IG_SURFACE_MESH) return false;
    auto input_mesh = DynamicCast<SurfaceMesh>(obj);
    if (input_mesh == nullptr) return false;
    auto mesh = SurfaceMesh::New();
    mesh->DeepCopy(input_mesh);
    mesh->RequestEditStatus();
    std::priority_queue<computedEdge, std::vector<computedEdge>,Compare> edgeQueue{};//等待坍缩的边的优先队列
    int numPoints = mesh->GetNumberOfPoints();
    std::vector<Eigen::Matrix4f> Quadrics(numPoints,Eigen::Matrix4f::Zero());//每个顶点的Quadrics
    int numEdge = mesh->GetNumberOfEdges();
    std::vector<int> version(numEdge, 0);
    ComputePointQ(mesh, Quadrics);//得到每个点的Quadrics
    for (int i = 0; i < numEdge; i++) {
        edgeQueue.push(ComputeEdge(i, mesh, version, Quadrics)); //初始化每条边，加入队列
    }

    int count = m_edgeCollapseNumber;
    while (count && !edgeQueue.empty()) {
        //得到正确的、error最小的边，对于已经不是最新version的边和已经被删除的边，要跳过
        computedEdge topEdge = edgeQueue.top(); 
        edgeQueue.pop();
        if (mesh->IsEdgeDeleted(topEdge.edgeId) || topEdge.version < version[topEdge.edgeId]) continue;
        else//找到了有效的边
            count--;
        
        auto edge = mesh->GetEdge(topEdge.edgeId);
        auto pointId1 = edge->GetPointId(0);
        auto pointId2 = edge->GetPointId(1);
        int pointId = -1;
        mesh->CollapseEdge(topEdge.edgeId);
        if (mesh->IsPointDeleted(pointId1)) {
            pointId = pointId2;
        } else if (mesh->IsPointDeleted(pointId2)) {
            pointId = pointId1;
        }//找到被合并到了哪个点
        mesh->SetPoint(pointId, topEdge.position);//优化点的位置
        Quadrics[pointId] = Quadrics[pointId1] + Quadrics[pointId2];//更新点的q
        UpdateEdges(edgeQueue, pointId, mesh, Quadrics,version);
    }
    mesh->GarbageCollection();
    SetOutput(mesh);
    return true;
}


IGAME_NAMESPACE_END