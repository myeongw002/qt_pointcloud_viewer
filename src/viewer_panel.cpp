#include "viewer_panel.hpp"


ViewerPanel::ViewerPanel(QWidget* parent)
    : QWidget(parent) {
    // Initialize the viewer
    viewer_ = new PointCloudWidget(this);
    
    
}


void ViewerPanel::setStatusLabel(QLabel* label) {
    label_ = label;
}

void ViewerPanel::setComboBox(QComboBox* comboBox) {
    comboBox_ = comboBox;
    // Connect signals and slots
    connect(comboBox_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ViewerPanel::onComboBoxIndexChanged);
}

void ViewerPanel::setPointCloudWidget(PointCloudWidget* viewer, rclcpp::Node::SharedPtr node) {
    viewer_ = viewer;
    if (node) {
        viewer_->setNode(node);
    }
}



void ViewerPanel::onComboBoxIndexChanged(int index) {
    // Update the topic name based on the selected index
    viewer_->setTopicName(index);
    updateStatus(QString("Selected Topic: %1").arg(viewer_->getTopicName().c_str()));
}


void ViewerPanel::updateStatus(const QString &status) {
        label_->setText(status);
        label_->adjustSize(); 
    }