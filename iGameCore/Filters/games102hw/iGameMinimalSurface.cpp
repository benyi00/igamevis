#include "iGameMinimalSurface.h"
#include <set>
#include <vector>

IGAME_NAMESPACE_BEGIN
bool myDotProduct(Face* face,Line*line,float &ans) {//输入面和一条边，计算另外两条边点乘
    igIndex p1=-1;
    igIndex p2=-1;
    igIndex p3=-1;
    for (int i = 0; i < 3; i++) {
        if (face->GetPointId(i) == line->GetPointId(0)) p2 = i;
        else if (face->GetPointId(i) == line->GetPointId(1)) p3 = i;
        else p1 = i;
    }
    if (p1 == -1 || p2 == -1 || p3 == -1) { 
        std::cerr << "计算点乘出错:找不到对应点。" << std::endl;
        return false; 
    }
    Vector3f v1 = face->GetPoint(p2) - face->GetPoint(p1);
    Vector3f v2 = face->GetPoint(p3) - face->GetPoint(p1);
    ans= DotProduct(v1, v2);
    return true;
}

bool myCrossProduct(Face* face, Line* line,float &ans) { //输入面和一条边，计算另外两条边叉乘,返回模
    igIndex p1=-1;
    igIndex p2=-1;
    igIndex p3=-1;
    for (int i = 0; i < 3; i++) {
        if (face->GetPointId(i) == line->GetPointId(0)) p2 = i;
        else if (face->GetPointId(i) == line->GetPointId(1))p3 = i;
        else p1 = i;
    }
    if (p1 == -1 || p2 == -1 || p3 == -1) {
        std::cerr << "计算叉乘出错:找不到对应点。" << std::endl;
        return false;
    }
    Vector3f v1 = face->GetPoint(p2) - face->GetPoint(p1);
    Vector3f v2 = face->GetPoint(p3) - face->GetPoint(p1);
    ans = CrossProduct(v1, v2).norm();
    return true;
}

bool computeW(SurfaceMesh::Pointer mesh,IGsize lineId,float& ans) {//计算权值cot\alpha_j +cot\beta_j。
    float cot[2]{};
    Line* line = mesh->GetEdge(lineId);
    igIndex fids[IGAME_CELL_MAX_SIZE];
    int faceSize = mesh->GetEdgeToNeighborFaces(lineId, fids);
    for (int i = 0; i < 2; i++) {//对两个面分别计算一下cot
        Face* face = mesh->GetFace(fids[i]);
        float dot = -1;
        if (!myDotProduct(face, line, dot)) return false;
        float cross = -1;
        if(!myCrossProduct(face, line,cross))return false;
        cot[i] = dot/cross;
    }
    ans= cot[0] + cot[1];
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

bool iteration(SurfaceMesh::Pointer mesh, igIndex p,Vector3f& ans) { //对一个点迭代一次
    float area = computeA(mesh, p);                    //计算面积
    int ids[IGAME_CELL_MAX_SIZE];
    Vector3f medianPos = Vector3f(0,0,0);
    Vector3f pointPos = mesh->GetPoint(p);
    int size = mesh->GetPointToOneRingPoints(p, ids); //得到周围一圈的点
    for (int i = 0; i < size; i++) {
        igIndex lineId = mesh->GetEdgeIdFormPointIds(p, ids[i]);
        Line* line = mesh->GetEdge(lineId);
        float w = -1;
        if (!computeW(mesh, lineId, w)) {
            std::cerr << "计算权重出错。" << std::endl;
            return false;
        }
        medianPos += w * (mesh->GetPoint(ids[i])-pointPos);
    }
    medianPos /= (4 * area);
    ans = medianPos;
    return true;
}

bool MinimalSurfaceFilter ::Execute() { 
    auto obj = GetInput(0); 
    if (obj == nullptr) {
        std::cerr << "Failed to get input obj" << std::endl;
        return false;
    }
    auto input = DynamicCast<SurfaceMesh>(obj);
    if (input == nullptr) {
        std::cerr << "Failed to get surface input" << std::endl;
        return false;
    }
    SurfaceMesh::Pointer surfaceInput = SurfaceMesh::New();
    surfaceInput->DeepCopy(input);
    surfaceInput->SetName(input->GetName() + "_Minimal");

    surfaceInput->BuildEdges();
    surfaceInput->BuildFaceLinks();
    surfaceInput->BuildEdgeLinks();
    surfaceInput->BuildFaceEdgeLinks();

    int numPoint = surfaceInput->GetNumberOfPoints();
    int numEdge = surfaceInput->GetNumberOfEdges();
    int numFace = surfaceInput->GetNumberOfFaces();
    
    /*首先要找边界点。
    * 找边界边。一条边只有一个相邻面就是边界边，上面的点就是边界点。*/
    std::set<igIndex> boundaryPoints;
    for (int i = 0; i < numEdge; i++)
    { 
        Line* edge = surfaceInput->GetEdge(i);
        igIndex adjFaceIds[IGAME_CELL_MAX_SIZE]{};
        int adjacentFacesNum = surfaceInput->GetEdgeToNeighborFaces(i, adjFaceIds);
        if (adjacentFacesNum == 1) { 
            boundaryPoints.insert(edge->GetPointId(0));
            boundaryPoints.insert(edge->GetPointId(1));
        } else if (adjacentFacesNum >= 3) {
            std::cerr << "不支持非流形" << std::endl;
            return false;
        }
    }

    std::vector<Vector3f> newPos(numPoint);//存放新的点坐标
    while (m_iteration_count--) {
        for (int p = 0; p < numPoint; p++) {
            if (boundaryPoints.count(p)) continue;//边界点不参与
            Vector3f w;
            if(!iteration(surfaceInput, p,w))return false;
            newPos[p] = surfaceInput->GetPoint(p)+m_lambda*(w);
        }
        for (int p = 0; p < numPoint; p++) {
            if (boundaryPoints.count(p)) continue;//边界点不参与
            surfaceInput->SetPoint(p, newPos[p]);
        }
    }
    
    SetOutput(surfaceInput);
    return true;
}

IGAME_NAMESPACE_END