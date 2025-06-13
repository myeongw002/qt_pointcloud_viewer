#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>      // ✅ 추가
#include <QTabWidget>       // ✅ 추가
#include <QHash>
#include <QAction>          // ✅ 추가
#include <QMenu>            // ✅ 추가
#include <QMenuBar>         // ✅ 추가
#include <QVector3D>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// 프로젝트 헤더들
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
#include "data_broker.hpp"
#include "control_tree_widget.hpp"
#include "debug_console_widget.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // ============================================================================
    // 🎛️ 기본 슬롯들
    // ============================================================================
    void openNewViewer();
    void onControlTabChanged(int index);
    
    // ============================================================================
    // 🖥️ 디버그 콘솔 슬롯들
    // ============================================================================
    void toggleDebugConsole();
    void showDebugConsole();
    void hideDebugConsole();

private:
    // ============================================================================
    // 🏗️ UI 구조 멤버들
    // ============================================================================
    Ui::MainWindow *ui_;
    
    // ============================================================================
    // 🤖 ROS2 관련 멤버들
    // ============================================================================
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<DataBroker> broker_;
    std::thread ros_thread_;
    
    // ============================================================================
    // 📺 뷰어 관련 멤버들
    // ============================================================================
    Widget::PointCloudWidget *viewer_;                                    // 메인 뷰어
    std::vector<Widget::ViewerPanel*> panels_;                           // 뷰어 패널들
    const int panelCount_ = 6;                                          // 패널 개수
    QHash<QString, Widget::PointCloudWidget*> pointCloudWidgets_;       // 로봇별 위젯들
    
    // ============================================================================
    // 🎛️ 제어 패널 관련 멤버들
    // ============================================================================
    QTabWidget* controlTabWidget_ = nullptr;                            // 제어 탭 위젯
    QDockWidget* controlDockWidget_ = nullptr;                          // 제어 독 위젯
    QHash<QString, Widget::ControlTreeWidget*> controlTrees_;           // 로봇별 TreeWidget들
    
    // ============================================================================
    // 🖥️ 디버그 콘솔 관련 멤버들
    // ============================================================================
    Widget::DebugConsoleWidget* debugConsole_ = nullptr;                // 디버그 콘솔 위젯
    QDockWidget* debugConsoleDock_ = nullptr;                           // 디버그 콘솔 독 위젯
    QAction* debugConsoleAction_ = nullptr;                             // 메뉴 액션
    
    // ============================================================================
    // 📋 메뉴 관련 멤버들
    // ============================================================================
    QMenu* fileMenu_ = nullptr;                                         // 파일 메뉴
    QMenu* viewMenu_ = nullptr;                                         // 뷰 메뉴
    QMenu* toolsMenu_ = nullptr;                                        // 도구 메뉴
    QMenu* helpMenu_ = nullptr;                                         // 도움말 메뉴
    
    // 파일 메뉴 액션들
    QAction* newViewerAction_ = nullptr;
    QAction* saveLogAction_ = nullptr;
    QAction* exitAction_ = nullptr;
    
    // 뷰 메뉴 액션들
    QAction* showControlPanelAction_ = nullptr;
    QAction* resetLayoutAction_ = nullptr;
    
    // 도구 메뉴 액션들
    QAction* resetAllCamerasAction_ = nullptr;
    QAction* resetAllColorsAction_ = nullptr;
    
    // ============================================================================
    // 🏗️ 초기화 함수들
    // ============================================================================
    void setupUI();                              // 전체 UI 설정
    void setupMenuBar();                         // 메뉴바 설정
    void setupStatusBar();                       // 상태바 설정
    void setupPointCloudWidgets();               // 포인트클라우드 위젯들 설정
    void setupViewerPanels();                    // 뷰어 패널들 설정
    void setupControlPanel();                    // 제어 패널 설정
    void setupDebugConsole();                    // 디버그 콘솔 설정
    
    // ============================================================================
    // 🔧 헬퍼 함수들
    // ============================================================================
    void createControlTrees();                   // 제어 트리들 생성
    void connectControlSignals();                // 제어 시그널 연결
    void connectMenuActions();                   // 메뉴 액션 연결
    
    // 위젯 검색 함수들
    Widget::PointCloudWidget* getWidgetByName(const QString& robotName);
    Widget::PointCloudWidget* findPointCloudWidget(const QString& objectName);
    
    // 레이아웃 관리 함수들
    void saveLayout();                           // 레이아웃 저장
    void restoreLayout();                        // 레이아웃 복원
    void resetToDefaultLayout();                 // 기본 레이아웃으로 리셋
    
    // ============================================================================
    // 🎯 유틸리티 함수들
    // ============================================================================
    void updateStatusBar(const QString& message);               // 상태바 업데이트
    void logToConsole(const QString& message, 
                     Widget::DebugConsoleWidget::LogLevel level = 
                     Widget::DebugConsoleWidget::INFO);         // 콘솔 로그
    
    // 설정 관리
    void loadSettings();                         // 설정 로드
    void saveSettings();                         // 설정 저장
    
protected:
    // ============================================================================
    // 🖥️ 이벤트 오버라이드
    // ============================================================================
    void closeEvent(QCloseEvent* event) override;               // 창 닫기 이벤트
    void resizeEvent(QResizeEvent* event) override;             // 크기 변경 이벤트
    void showEvent(QShowEvent* event) override;                 // 표시 이벤트
    void keyPressEvent(QKeyEvent* event) override;              // 키 입력 이벤트
};

#endif // MAINWINDOW_H
