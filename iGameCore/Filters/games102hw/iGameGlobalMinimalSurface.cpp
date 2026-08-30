#include "iGameGlobalMinimalSurface.h"
#include <Eigen/Sparse>
#include <set>

IGAME_NAMESPACE_BEGIN
namespace
{
bool myDotProduct(Face* face, Line* line, float& ans) { //输入面和一条边，计算另外两条边点乘
    igIndex p1 = -1;
    igIndex p2 = -1;
    igIndex p3 = -1;
    for (int i = 0; i < 3; i++) {
        if (face->GetPointId(i) == line->GetPointId(0)) p2 = i;
        else if (face->GetPointId(i) == line->GetPointId(1))
            p3 = i;
        else
            p1 = i;
    }
    if (p1 == -1 || p2 == -1 || p3 == -1) {
        std::cerr << "Failed to compute dot" << std::endl;
        return false;
    }
    Vector3f v1 = face->GetPoint(p2) - face->GetPoint(p1);
    Vector3f v2 = face->GetPoint(p3) - face->GetPoint(p1);
    ans = DotProduct(v1, v2);
    return true;
}

bool myCrossProduct(Face* face, Line* line, float& ans) { //输入面和一条边，计算另外两条边叉乘,返回模
    igIndex p1 = -1;
    igIndex p2 = -1;
    igIndex p3 = -1;
    for (int i = 0; i < 3; i++) {
        if (face->GetPointId(i) == line->GetPointId(0)) p2 = i;
        else if (face->GetPointId(i) == line->GetPointId(1))
            p3 = i;
        else
            p1 = i;
    }
    if (p1 == -1 || p2 == -1 || p3 == -1) {
        std::cerr << "Failed to compute cross" << std::endl;
        return false;
    }
    Vector3f v1 = face->GetPoint(p2) - face->GetPoint(p1);
    Vector3f v2 = face->GetPoint(p3) - face->GetPoint(p1);
    ans = CrossProduct(v1, v2).norm();
    return true;
}

bool computeW(SurfaceMesh::Pointer mesh, IGsize lineId, float& ans) { //计算权值cot\alpha_j +cot\beta_j。
    float cot[2]{};
    Line* line = mesh->GetEdge(lineId);
    igIndex fids[IGAME_CELL_MAX_SIZE];
    int faceSize = mesh->GetEdgeToNeighborFaces(lineId, fids);
    for (int i = 0; i < faceSize; i++) { //对两个面分别计算一下cot
        Face* face = mesh->GetFace(fids[i]);
        float dot = -1;
        if (!myDotProduct(face, line, dot)) return false;
        float cross = -1;
        if (!myCrossProduct(face, line, cross)) return false;
        cot[i] = dot / cross;
    }
    ans = cot[0] + cot[1];
    return true;
}

float computeA(SurfaceMesh::Pointer mesh, igIndex p) { //计算一个点周围1邻域范围的面积
    igIndex faces[IGAME_CELL_MAX_SIZE];
    int count = mesh->GetPointToNeighborFaces(p, faces);
    float ans = 0;
    for (int i = 0; i < count; i++) {
        auto face = mesh->GetFace(faces[i]);
        Vector3f v1 = face->GetPoint(0) - face->GetPoint(1);
        Vector3f v2 = face->GetPoint(2) - face->GetPoint(1);
        ans += 0.5 * CrossProduct(v1, v2).norm();
    }
    return ans;
}

struct adjNode {
    igIndex id;
    Point pos;
    adjNode* next;
};

} // namespace

bool GlobalMinimalSurfaceFilter::Execute() {
    auto obj = GetInput(0);
    if (obj == nullptr) {
        std::cerr << "Failed to get obj!" << std::endl;
        return false;
    }
    auto surfaceInput = DynamicCast<SurfaceMesh>(obj);
    if (surfaceInput == nullptr) {
        std::cerr << "Failed to get surface mesh!" << std::endl;
        return false;
    }

    surfaceInput->BuildEdges();
    surfaceInput->BuildEdgeLinks();
    surfaceInput->BuildFaceLinks();
    surfaceInput->BuildFaceEdgeLinks();

    IGsize numEdge = surfaceInput->GetNumberOfEdges();
    /*首先要找边界点。
    * 找边界边。一条边只有一个相邻面就是边界边，上面的点就是边界点。*/
    std::set<igIndex> boundaryPoints;
    for (int i = 0; i < numEdge; i++) {
        Line* edge = surfaceInput->GetEdge(i);
        igIndex adjFaceIds[IGAME_CELL_MAX_SIZE]{};
        int adjacentFacesNum = surfaceInput->GetEdgeToNeighborFaces(i, adjFaceIds);
        if (adjacentFacesNum == 1) {
            boundaryPoints.insert(edge->GetPointId(0));
            boundaryPoints.insert(edge->GetPointId(1));
        } else if (adjacentFacesNum >= 3) {
            std::cerr << "not support no-manifold" << std::endl;
            return false;
        }
    }
    std::cout << boundaryPoints.size() << std::endl;
    adjNode* firstNode{}; //建立了一个链表
    std::set<igIndex> visitedPoint;
    int num = 0;
    if (m_parameterization) {//如果不参数化就不需要按顺序整理了
        adjNode* node=new adjNode;
        bool flag = false;
        while(1) {
            if (!flag) {
                node->id = *boundaryPoints.begin();
                firstNode = node;
                flag = true;
                num++;
            }
            node->pos = surfaceInput->GetPoint(node->id);
            visitedPoint.insert(node->id);
            igIndex ids[IGAME_CELL_MAX_SIZE];
            IGsize size = surfaceInput->GetPointToOneRingPoints(node->id, ids);
            bool findNext = false;
            for (int i = 0; i < size; i++) { 
                if (visitedPoint.count(ids[i]) || boundaryPoints.count(ids[i])==0) continue;
                node->next = new adjNode;
                node->next->id = ids[i];
                num++;
                findNext = true;
                break;
            }
            if (!findNext) {
                node->next = nullptr;
                break; //所有点都找到了
            }
            node = node->next;
        }
    }

    //遍历每一个点，构造L
    IGsize numPoints = surfaceInput->GetNumberOfPoints();
    Eigen::SparseMatrix<double> L(numPoints + boundaryPoints.size(), numPoints);
    using T = Eigen::Triplet<double>;
    std::vector<T> triplets;
    for (igIndex p = 0; p < numPoints; p++) { //遍历每一个点
        igIndex neighborIds[IGAME_CELL_MAX_SIZE];
        IGsize size = surfaceInput->GetPointToOneRingPoints(p, neighborIds);
        if (size > IGAME_CELL_MAX_SIZE) {
            std::cerr << "too many one ring points" << std::endl;
            return false;
        }
        float sumW = 0;
        float A = computeA(surfaceInput, p);
        for (int i = 0; i < size; i++) { //遍历每一个相邻的点
            igIndex adjPoint = neighborIds[i];
            igIndex lineId = surfaceInput->GetEdgeIdFormPointIds(p, adjPoint);
            float w = 0;
            if (!computeW(surfaceInput, lineId, w)) {
                std::cerr << "Falied to compute W on point "<<p<< std::endl;
                return false;
            }
            sumW += w;
            triplets.emplace_back(p, adjPoint, (double) - w / (4 * A));
        }
        triplets.emplace_back(p, p, sumW/(4*A));
    }
    std::vector<double> bx(numPoints, 0);
    std::vector<double> by(numPoints, 0);
    std::vector<double> bz(numPoints, 0);
    //增加约束
    if (!m_parameterization) {
        int count = 0;
        for (auto p: boundaryPoints) {
            Vector3f pointPos = surfaceInput->GetPoint(p);
            triplets.emplace_back(numPoints + count, p, 1);
            count++;
            bx.push_back(pointPos[0]);
            by.push_back(pointPos[1]);
            bz.push_back(pointPos[2]);
        }
    }
    if (m_parameterization) {
        float angle = 2*IGM_PI / (boundaryPoints.size() + 1);
        adjNode* node=firstNode;
        int count = 0;
        while (node != nullptr) {
            triplets.emplace_back(numPoints + count, node->id, 1);
            node = node->next;
            double x = (double)10*cos(angle * count);
            double y = (double)10*sin(angle * count);
            bx.push_back(x);
            by.push_back(y);
            bz.push_back(0);
            count++;
        }
    }
    L.setFromTriplets(triplets.begin(), triplets.end());
    //最小二乘
    Eigen::VectorXd vbx = Eigen::Map<Eigen::VectorXd>(bx.data(), bx.size());
    Eigen::VectorXd vby = Eigen::Map<Eigen::VectorXd>(by.data(), by.size());
    Eigen::VectorXd vbz = Eigen::Map<Eigen::VectorXd>(bz.data(), bz.size());
    

    Eigen::VectorXd vx;
    Eigen::VectorXd vy;
    Eigen::VectorXd vz;
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(L);
    vx = solver.solve(vbx);
    vy = solver.solve(vby);
    vz = solver.solve(vbz);
    SurfaceMesh::Pointer outputMesh = SurfaceMesh::New();
    outputMesh->DeepCopy(surfaceInput);
    for (igIndex i = 0; i < numPoints; i++) {
        Point point = Vector3f(vx[i], vy[i], vz[i]);
        outputMesh->SetPoint(i, point);
    }
    SetOutput(outputMesh);
    return true;
}

IGAME_NAMESPACE_END