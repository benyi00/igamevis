#ifndef iGameGlobalMinimalSurface_h
#define iGameGlobalMinimalSurface_h
#include "iGameFilter.h"
#include "iGamePointSet.h"
#include "iGameSurfaceMesh.h"
#include "iGameUnstructuredMesh.h"

IGAME_NAMESPACE_BEGIN
class GlobalMinimalSurfaceFilter : public Filter {
public:
    I_OBJECT(GlobalMinimalSurfaceFilter);
    static Pointer New() { return new GlobalMinimalSurfaceFilter; }
    bool Execute() override;
    void SetParameterization(bool flag) { m_parameterization = flag; }

protected:
    GlobalMinimalSurfaceFilter() {
        this->SetNumberOfInputs(1);
        this->SetNumberOfOutputs(1);
    }

    bool m_parameterization;
};



IGAME_NAMESPACE_END
#endif // !iGameGlobalMinimalSurface_h
