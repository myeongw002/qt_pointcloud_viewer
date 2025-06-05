#pragma once

#include <QWidget>      
#include <QComboBox>    
#include <QLabel>        
#include <QString>       
#include <string>       
#include <QCheckBox>
#include "pointcloud_widget.hpp" 
#include "float_widget.hpp"

namespace Widget{
    class ViewerPanel : public QWidget {
        Q_OBJECT
    
    public:
        ViewerPanel(QWidget* parent = nullptr);
        std::string getSelectedTopic() const;
        void setStatusLabel(QLabel* label);
        void setRobotComboBox(QComboBox* comboBox);
        void setPointCloudWidget(PointCloudWidget* viewer, rclcpp::Node::SharedPtr node = nullptr); 
        void setFloatWidget(FloatWidget* floatWidget);
        void setAxisCheckBox(QCheckBox* checkBox);
        void setGridCheckBox(QCheckBox* checkBox);
        void setPanelIdx_(int idx);
        PointCloudWidget* getPointCloudWidget() const;
        FloatWidget* getFloatWidget() const;
    
    private slots:
        //void startStreaming();
        void updateStatus(const QString &status);
        void onComboBoxIndexChanged(int index);   
        void onAxisCheckBoxToggled(bool checked);
        void onGridCheckBoxToggled(bool checked); 
    
        private:
        PointCloudWidget* viewer_;
        FloatWidget* floatWidget_;

        QComboBox* comboBox_;
        QLabel* label_;
        int panelIdx_ = 0; // Index for the display panel
    };    
}
