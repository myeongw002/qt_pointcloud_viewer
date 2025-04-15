#pragma once

#include <QWidget>      
#include <QComboBox>    
#include <QLabel>        
#include <QString>       
#include <string>       
#include "pointcloud_widget.hpp" 

class ViewerPanel : public QWidget {
    Q_OBJECT

public:
    ViewerPanel(QWidget* parent = nullptr);
    std::string getSelectedTopic() const;
    void setStatusLabel(QLabel* label);
    void setComboBox(QComboBox* comboBox);
    void setPointCloudWidget(PointCloudWidget* viewer, rclcpp::Node::SharedPtr node = nullptr); 
    

private slots:
    //void startStreaming();
    void updateStatus(const QString &status);
    void onComboBoxIndexChanged(int index);    

    private:
    PointCloudWidget* viewer_;
    QComboBox* comboBox_;
    QLabel* label_;
};
