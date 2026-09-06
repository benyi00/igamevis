#ifndef iGameQEM_h
#define iGameQEM_h
#include "iGameFilter.h"
#include "iGamePointSet.h"
#include "iGameSurfaceMesh.h"
#include "iGameUnstructuredMesh.h"

IGAME_NAMESPACE_BEGIN
class QEMFilter : public Filter {
public:
    I_OBJECT(QEMFilter);
    static Pointer New() { return new QEMFilter(); }

    bool Execute() override;

    void SetEdgeCollapseNumber(int num) { m_edgeCollapseNumber = num; };

protected:
    QEMFilter() {
        this->SetNumberOfInputs(1);
        this->SetNumberOfOutputs(1);
    }
    
    int m_edgeCollapseNumber=0;
};

IGAME_NAMESPACE_END
#endif // iGameQEM_h
