#include "viewer_panel.hpp"

namespace Widget {
    ViewerPanel::ViewerPanel(QWidget* parent)
    : QWidget(parent) {
    }

    void ViewerPanel::setStatusLabel(QLabel* label) {
        label_ = label;
    }

    void ViewerPanel::setPointCloudWidget(PointCloudWidget* viewer) {
        viewer_ = viewer;
    }

    void ViewerPanel::setAxisCheckBox(QCheckBox* checkBox) {
        // Connect signals and slots for axes visibility
        connect(checkBox, &QCheckBox::toggled, viewer_, &PointCloudWidget::setShowAxes);
    }

    void ViewerPanel::setGridCheckBox(QCheckBox* checkBox) {
        // Connect signals and slots for grid visibility
        connect(checkBox, &QCheckBox::toggled, viewer_, &PointCloudWidget::setShowGrid);
    }
    
    void ViewerPanel::onAxisCheckBoxToggled(bool checked) {
        viewer_->setShowAxes(checked);
        updateStatus(QString("Axes %1").arg(checked ? "Enabled" : "Disabled"));
    }

    void ViewerPanel::onGridCheckBoxToggled(bool checked) {
        viewer_->setShowGrid(checked);
        updateStatus(QString("Grid %1").arg(checked ? "Enabled" : "Disabled"));
    }


    void ViewerPanel::updateStatus(const QString &status) {
            label_->setText(status);
            label_->adjustSize(); 
        }

    void ViewerPanel::setPanelIdx_(int idx) {
        panelIdx_ = idx;
    }    
}
