#include "IQCore/igQtMainWindow.h"
#include <IQComponents/igQtFilterDialogDockWidget.h>
#include <IQComponents/igQtModelDialogWidget.h>
#include <IQWidgets/igQtModelDrawWidget.h>
#include <games102hw/iGameMinimalSurface.h>
#include <games102hw/iGameGlobalMinimalSurface.h>
#include <games102hw/iGameQEM.h>
#include <iGameScene.h>
#include <QAction>
#include <QMenu>
#include <cmath>

using namespace iGame;

void igQtMainWindow::initGames102Homework() {
    auto currentSurfaceInput = [this]() -> SurfaceMesh::Pointer {
        auto scene = rendererWidget->GetScene();
        if (!scene || !scene->GetCurrentModel()) {
            showDarkFramelessMessage(QStringLiteral("Games102作业"), QStringLiteral("请先加载并选择一个模型。"));
            return nullptr;
        }
        auto input = DynamicCast<SurfaceMesh>(scene->GetCurrentModel()->GetDataObject());
        if (!input) {
            showDarkFramelessMessage(QStringLiteral("Games102作业"), QStringLiteral("请先转换为表面网格。"));
        }
        return input;
    };

    QMenu* Games102hw = ui->menu_filters->addMenu(QStringLiteral("Games102作业"));
    QAction* minimalSurface = Games102hw->addAction(QStringLiteral("hw6 极小曲面"));
    connect(minimalSurface, &QAction::triggered, this, [this, currentSurfaceInput](bool) {
        auto input = currentSurfaceInput();

        if (!input) return;
        const QString title = QStringLiteral("hw6 极小曲面");
        MinimalSurfaceFilter::Pointer filter = MinimalSurfaceFilter::New();
        igQtFilterDialogDockWidget* dialog = new igQtFilterDialogDockWidget(this, true);
        int lambda = dialog->addParameter(igQtFilterDialogDockWidget::QT_LINE_EDIT, QStringLiteral("移动比例lambda (0..1)"), "0.1");
        int iteration_count = dialog->addParameter(igQtFilterDialogDockWidget::QT_LINE_EDIT, QStringLiteral("迭代次数"), "8");

        dialog->setFilterTitle(title);
        dialog->show();
        dialog->setApplyFunctor([=, this]() {
            bool ok;
            float lambdaValue = dialog->getDouble(lambda, ok);
            if (!ok || !std::isfinite(lambdaValue) || lambdaValue >= 1 || lambdaValue <= 0) {
                showDarkFramelessMessage(QStringLiteral("Warning"), QStringLiteral("请输入正常范围内的参数（0到1）。"));
                return;
            }
            filter->SetLambda(lambdaValue);
            int iterationValue = dialog->getInt(iteration_count, ok);
            if (!ok || iterationValue < 1) {
                showDarkFramelessMessage(QStringLiteral("Warning"),
                                         QStringLiteral("请输入正常范围内的参数（至少为1）。"));
                return;
            }
            filter->SetIterationCount(iterationValue);

            filter->SetInput(input);
            if (!filter->Execute()) {
                showDarkFramelessMessage(QStringLiteral("Warning"),
                                         QStringLiteral("执行失败。"));
                return;
            }
            auto newMesh = filter->GetOutput();
            if (!newMesh) {
                showDarkFramelessMessage(title, QStringLiteral("Filter 未产生有效输出。"));
                return;
            }
            modelTreeWidget->addDataObjectToModelTree(newMesh, Algorithm);
            rendererWidget->update();
            dialog->close();
        });
    });

    QAction* globalMinimalSurface = Games102hw->addAction(QStringLiteral("hw7 global极小曲面"));
    connect(globalMinimalSurface,&QAction::triggered,this, [this, currentSurfaceInput](bool) {
        auto input = currentSurfaceInput();
        if (!input) return;
        const QString title = QStringLiteral("hw7 global极小曲面");
        GlobalMinimalSurfaceFilter::Pointer filter = GlobalMinimalSurfaceFilter::New();

        igQtFilterDialogDockWidget* dialog = new igQtFilterDialogDockWidget(this, true);

        int parameterizationId = dialog->addParameter(igQtFilterDialogDockWidget::QT_CHECK_BOX, QStringLiteral("是否参数化"), "false");
        dialog->setFilterTitle(title);
        dialog->show();
        dialog->setApplyFunctor([=, this]() {
            bool ok;
            bool parameterization = dialog->getChecked(parameterizationId, ok);
            if (!ok) {
                showDarkFramelessMessage(QStringLiteral("Warning"), QStringLiteral("请输入是否参数化"));
                return;
            }
            filter->SetParameterization(parameterization);
            filter->SetInput(input);
            if (!filter->Execute()) {
                showDarkFramelessMessage(QStringLiteral("Warning"), QStringLiteral("执行失败。"));
                return;
            }
            auto newMesh = filter->GetOutput();
            if (!newMesh) {
                showDarkFramelessMessage(title, QStringLiteral("Filter 未产生有效输出。"));
                return;
            }
            modelTreeWidget->addDataObjectToModelTree(newMesh, Algorithm);
            rendererWidget->update();
            dialog->close();
        });

    });

    QAction* QEM = Games102hw->addAction(QStringLiteral("hw9 QEM"));
    connect(QEM, &QAction::triggered, this,[this, currentSurfaceInput](bool) {
        auto input = currentSurfaceInput();
        if (!input) return;
        const QString title = QStringLiteral("hw9 QEM");
        QEMFilter::Pointer filter = QEMFilter::New();
        igQtFilterDialogDockWidget* dialog = new igQtFilterDialogDockWidget(this, true);
        int edgeCollapseNumberId = dialog->addParameter(igQtFilterDialogDockWidget::QT_LINE_EDIT, QStringLiteral("坍缩边数量"), "200");
        dialog->setFilterTitle(title);
        dialog->show();
        dialog->setApplyFunctor([=, this]() {
            bool ok;
            int edgeCollapseNumber = dialog->getInt(edgeCollapseNumberId,ok);
            if (!ok || edgeCollapseNumber < 0) {
                showDarkFramelessMessage(QStringLiteral("Warning"), QStringLiteral("请输入非负整数。"));
                return;
            }
            filter->SetEdgeCollapseNumber(edgeCollapseNumber);
            filter->SetInput(input);
            if (!filter->Execute()) {
                showDarkFramelessMessage(QStringLiteral("Warning"), QStringLiteral("执行失败。"));
                return;
            }
            auto newMesh = filter->GetOutput();
            if (!newMesh) {
                showDarkFramelessMessage(title, QStringLiteral("Filter 未产生有效输出。"));
                return;
            }
            modelTreeWidget->addDataObjectToModelTree(newMesh, Algorithm);
            rendererWidget->update();
            dialog->close();
        });
    });
}
