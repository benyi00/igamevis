#ifndef iGameMinimalSurface_h
#define iGameMinimalSurface_h
#include "iGameFilter.h"
#include "iGamePointSet.h"
#include "iGameSurfaceMesh.h"
#include "iGameUnstructuredMesh.h"

IGAME_NAMESPACE_BEGIN
class MinimalSurfaceFilter : public Filter {
public:
    I_OBJECT(MinimalSurfaceFilter);
    static Pointer New() { return new MinimalSurfaceFilter; }

    bool Execute() override;

    void SetLambda(float lambda) { m_lambda = lambda;}
    void SetIterationCount(int count) { m_iteration_count = count; }

protected:
    MinimalSurfaceFilter() { 
        this->SetNumberOfInputs(1);
        this->SetNumberOfOutputs(1);
    }
    float m_lambda=0.1f;//比例参数
    int m_iteration_count=8;//迭代次数
};

IGAME_NAMESPACE_END
#endif