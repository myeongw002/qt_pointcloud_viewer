#pragma once

#include <QWidget>      
#include <QComboBox>    
#include <QLabel>        
#include <QString>       
#include <string>       
#include <QCheckBox>
#include "pointcloud_widget.hpp" 


namespace Widget{
    class ViewerPanel : public QWidget {
        Q_OBJECT
    
    public:
        ViewerPanel(QWidget* parent = nullptr);
        std::string getSelectedTopic() const;
        void setStatusLabel(QLabel* label);
        void setPointCloudWidget(PointCloudWidget* viewer, rclcpp::Node::SharedPtr node = nullptr); 
        void setAxisCheckBox(QCheckBox* checkBox);
        void setGridCheckBox(QCheckBox* checkBox);
        void setPanelIdx_(int idx);
    
    private slots:
        //void startStreaming();
        void updateStatus(const QString &status);
        void onAxisCheckBoxToggled(bool checked);
        void onGridCheckBoxToggled(bool checked); 
    
        private:
        PointCloudWidget* viewer_;
        QComboBox* comboBox_;
        QLabel* label_;
        int panelIdx_ = 0; // Index for the display panel
    };    
}
