#include "theme_manager.hpp"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QDebug>
#include <QStandardPaths>
#include <QDir>
#include <ament_index_cpp/get_package_share_directory.hpp>  // 추가

namespace Theme {

ThemeManager* ThemeManager::instance_ = nullptr;

ThemeManager* ThemeManager::instance() {
    if (!instance_) {
        instance_ = new ThemeManager();
    }
    return instance_;
}

ThemeManager::ThemeManager(QObject* parent)
    : QObject(parent), currentThemeName_("Default") {
    initializeDefaultThemes();
    
    // ROS2 패키지 경로에서 themes.json 로드 시도
    try {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("qt_pointcloud_viewer");
        QString themeFilePath = QString::fromStdString(package_share_dir) + "/resources/themes.json";
        
        qDebug() << "Looking for theme file at:" << themeFilePath;
        
        if (QFile::exists(themeFilePath)) {
            loadThemesFromFile(themeFilePath);
        } else {
            qDebug() << "Theme file not found at:" << themeFilePath;
            qDebug() << "Using default themes only";
        }
    } catch (const std::exception& e) {
        qWarning() << "Failed to get package directory:" << e.what();
        qDebug() << "Using default themes only";
    }
}

void ThemeManager::initializeDefaultThemes() {
    // 기본 테마만 유지 (현재 스타일)
    themes_["Default"] = ThemeData{
        "Default",
        {"#F0F0F0", "#E0E0E0", "#0078D4", "#000000"}
    };
}

bool ThemeManager::loadThemesFromFile(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot open theme file:" << filePath;
        return false;
    }
    
    QByteArray data = file.readAll();
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "JSON parse error:" << error.errorString();
        return false;
    }
    
    QJsonObject root = doc.object();
    
    for (auto it = root.begin(); it != root.end(); ++it) {
        QString themeKey = it.key();
        QJsonObject themeObj = it.value().toObject();
        
        ThemeData theme;
        theme.name = themeObj["name"].toString();
        theme.colors.primary = themeObj["color1"].toString();
        theme.colors.secondary = themeObj["color2"].toString();
        theme.colors.accent = themeObj["color3"].toString();
        theme.colors.text = themeObj["color4"].toString();
        
        themes_[themeKey] = theme;
        qDebug() << "Loaded theme:" << theme.name;
    }
    
    return true;
}

QStringList ThemeManager::getAvailableThemes() const {
    QStringList themeNames;
    
    // 1. Default 테마를 먼저 추가
    if (themes_.contains("Default")) {
        themeNames << themes_["Default"].name;
    }
    
    // 2. JSON에서 로드된 테마들만 추가 (theme1, theme2, ... 순서)
    QStringList jsonKeys;
    for (auto it = themes_.begin(); it != themes_.end(); ++it) {
        QString key = it.key();
        if (key != "Default") {  // Default가 아닌 모든 테마 (JSON에서 로드된 것들)
            jsonKeys << key;
        }
    }
    
    // JSON 키를 정렬 (theme1, theme2, theme3... 순서)
    std::sort(jsonKeys.begin(), jsonKeys.end(), [](const QString& a, const QString& b) {
        // "theme" 접두사를 제거하고 숫자로 비교
        if (a.startsWith("theme") && b.startsWith("theme")) {
            bool aOk, bOk;
            int aNum = a.mid(5).toInt(&aOk);  // "theme" 이후 부분을 숫자로 변환
            int bNum = b.mid(5).toInt(&bOk);
            if (aOk && bOk) {
                return aNum < bNum;
            }
        }
        return a < b;  // 기본 문자열 정렬
    });
    
    // 정렬된 JSON 테마들을 리스트에 추가
    for (const QString& key : jsonKeys) {
        if (themes_.contains(key)) {
            themeNames << themes_[key].name;
        }
    }
    
    return themeNames;
}

ThemeData ThemeManager::getCurrentTheme() const {
    return themes_.value(currentThemeName_);
}

void ThemeManager::applyTheme(const QString& themeName) {
    // 테마 이름으로 키 찾기
    QString themeKey;
    for (auto it = themes_.begin(); it != themes_.end(); ++it) {
        if (it.value().name == themeName) {
            themeKey = it.key();
            break;
        }
    }
    
    if (themeKey.isEmpty() || !themes_.contains(themeKey)) {
        qWarning() << "Theme not found:" << themeName;
        return;
    }
    
    currentThemeName_ = themeKey;
    ThemeData theme = themes_[themeKey];
    
    QString styleSheet = generateStyleSheet(theme.colors);
    qApp->setStyleSheet(styleSheet);
    
    qDebug() << "Applied theme:" << theme.name;
    if (theme.name == "Default") {
        applyDefaultTheme();  // 기본 테마는 스타일시트 제거
    } else {
    emit themeChanged(themeName);    
    }
    
}

void ThemeManager::applyDefaultTheme() {
    // 완전히 스타일시트 제거하여 Qt 기본 스타일로 복원
    qApp->setStyleSheet("");
    currentThemeName_ = "Default";
    
    // 강제로 스타일 업데이트
    qApp->processEvents();
    
    qDebug() << "Reset to default theme";
    emit themeChanged("Default");
}

QString ThemeManager::generateStyleSheet(const ThemeColors& colors) {
    // 색상 변형 생성 (밝기 조절)
    QColor primaryColor(colors.primary);
    QColor secondaryColor(colors.secondary);
    QColor accentColor(colors.accent);
    QColor textColor(colors.text);
    
    // 다양한 밝기의 색상 생성
    QString primaryLight = primaryColor.lighter(120).name();
    QString primaryDark = primaryColor.darker(120).name();
    QString secondaryLight = secondaryColor.lighter(120).name();
    QString secondaryDark = secondaryColor.darker(120).name();
    QString accentLight = accentColor.lighter(150).name();
    QString accentDark = accentColor.darker(150).name();
    QString textMuted = textColor.darker(150).name();
    
    return QString(R"(
        /* ============================================================================
           메인 윈도우 및 기본 컨테이너
        ============================================================================ */
        QMainWindow {
            background-color: %1;
            color: %4;
            font-family: "Segoe UI", "Arial", sans-serif;
        }
        
        QWidget {
            background-color: %1;
            color: %4;
            font-size: 9pt;
        }
        
        /* ============================================================================
           메뉴바 - 세련된 플랫 디자인
        ============================================================================ */
        QMenuBar {
            background-color: %2;
            color: %4;
            border: none;
            border-bottom: 2px solid %8;
            padding: 2px;
            font-weight: 500;
        }
        
        QMenuBar::item {
            background-color: transparent;
            padding: 8px 12px;
            margin: 10px;
            border-radius: 4px;
            font-weight: 500;
        }
        
        QMenuBar::item:selected {
            background-color: %3;
            color: %1;
        }
        
        QMenuBar::item:pressed {
            background-color: %9;
            color: %1;
        }
        
        /* ============================================================================
           메뉴 드롭다운 - 그림자 효과와 부드러운 전환
        ============================================================================ */
        QMenu {
            background-color: %1;
            color: %4;
            border: 1px solid %8;
            border-radius: 6px;
            padding: 4px;
            font-size: 9pt;
        }
        
        QMenu::item {
            background-color: transparent;
            padding: 8px 24px 8px 8px;
            margin: 1px;
            border-radius: 4px;
        }
        
        QMenu::item:selected {
            background-color: %3;
            color: %1;
        }
        
        QMenu::item:disabled {
            color: %10;
            background-color: transparent;
        }
        
        QMenu::separator {
            height: 1px;
            background-color: %8;
            margin: 4px 8px;
        }
        
        QMenu::indicator {
            width: 16px;
            height: 16px;
            margin-left: 4px;
        }
        
        /* ============================================================================
           독 위젯 - 모던한 제목바와 드래그 핸들
        ============================================================================ */
        QDockWidget {
            background-color: %1;
            color: %4;
            border: 1px solid %8;
            border-radius: 6px;
            titlebar-close-icon: none;
            titlebar-normal-icon: none;
        }
        
        QDockWidget::title {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %7, stop:1 %2);
            color: %4;
            padding: 8px;
            border: none;
            border-top-left-radius: 6px;
            border-top-right-radius: 6px;
            font-weight: 600;
            text-align: center;
        }
        
        QDockWidget::close-button, QDockWidget::float-button {
            background-color: transparent;
            border: none;
            border-radius: 3px;
            padding: 2px;
            icon-size: 12px;
        }
        
        QDockWidget::close-button:hover, QDockWidget::float-button:hover {
            background-color: %3;
        }
        
        /* ============================================================================
           탭 위젯 - 세련된 탭 디자인
        ============================================================================ */
        QTabWidget {
            background-color: %1;
            border: none;
        }
        
        QTabWidget::pane {
            border: 1px solid %8;
            border-radius: 6px;
            background-color: %1;
            padding: 4px;
        }
        
        QTabWidget::tab-bar {
            alignment: left;
        }
        
        QTabBar {
            background-color: transparent;
        }
        
        QTabBar::tab {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %7, stop:1 %2);
            color: %4;
            padding: 10px 16px;
            margin-right: 2px;
            margin-bottom: 2px;
            border: 1px solid %8;
            border-top-left-radius: 6px;
            border-top-right-radius: 6px;
            font-weight: 500;
            min-width: 80px;
        }
        
        QTabBar::tab:selected {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %11, stop:1 %3);
            color: %1;
            border-bottom: 2px solid %3;
            font-weight: 600;
        }
        
        QTabBar::tab:hover:!selected {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %7, stop:1 %8);
        }
        
        QTabBar::tab:disabled {
            color: %10;
            background-color: %8;
        }
        
        /* ============================================================================
           트리 위젯 - 계층 구조 시각화
        ============================================================================ */
        QTreeWidget {
            background-color: %1;
            color: %4;
            border: 1px solid %8;
            border-radius: 6px;
            alternate-background-color: %7;
            selection-background-color: %3;
            outline: none;
            gridline-color: %8;
            show-decoration-selected: 1;
        }
        
        QTreeWidget::item {
            padding: 6px;
            border: none;
            border-bottom: 1px solid transparent;
        }
        
        QTreeWidget::item:selected {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %11, stop:1 %3);
            color: %1;
            border-radius: 3px;
        }
        
        QTreeWidget::item:hover:!selected {
            background-color: %7;
            border-radius: 3px;
        }
        
        QTreeWidget::branch {
            background-color: transparent;
        }
        
        QTreeWidget::branch:has-children:!has-siblings:closed,
        QTreeWidget::branch:closed:has-children:has-siblings {
            border-image: none;
            image: url(:/icons/branch_closed.png);
        }
        
        QTreeWidget::branch:open:has-children:!has-siblings,
        QTreeWidget::branch:open:has-children:has-siblings {
            border-image: none;
            image: url(:/icons/branch_open.png);
        }
        
        /* ============================================================================
           버튼 - 다양한 상태의 세밀한 스타일링
        ============================================================================ */
        QPushButton {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %7, stop:1 %2);
            color: %4;
            border: 1px solid %8;
            padding: 8px 16px;
            border-radius: 6px;
            font-weight: 500;
            min-width: 80px;
        }
        
        QPushButton:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %11, stop:1 %3);
            color: %1;
            border: 1px solid %3;
        }
        
        QPushButton:pressed {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %9, stop:1 %9);
            color: %1;
            border: 2px solid %9;
            padding: 9px 15px 7px 17px; /* 눌린 효과 */
        }
        
        QPushButton:disabled {
            background-color: %8;
            color: %10;
            border: 1px solid %8;
        }
        
        QPushButton:default {
            border: 2px solid %3;
            font-weight: 600;
        }
        
        /* ============================================================================
           텍스트 편집 위젯 - 깔끔한 입력 필드
        ============================================================================ */
        QTextEdit, QPlainTextEdit {
            background-color: %1;
            color: %4;
            border: 2px solid %8;
            border-radius: 6px;
            padding: 8px;
            selection-background-color: %3;
            selection-color: %1;
            font-family: "Consolas", "Monaco", monospace;
            font-size: 9pt;
        }
        
        QTextEdit:focus, QPlainTextEdit:focus {
            border: 2px solid %3;
        }
        
        QTextEdit:disabled, QPlainTextEdit:disabled {
            background-color: %8;
            color: %10;
        }
        
        /* ============================================================================
           콤보박스 - 드롭다운 스타일링
        ============================================================================ */
        QComboBox {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %7, stop:1 %2);
            color: %4;
            border: 1px solid %8;
            border-radius: 6px;
            padding: 6px 30px 6px 10px;
            min-width: 100px;
        }
        
        QComboBox:hover {
            border: 1px solid %3;
        }
        
        QComboBox:focus {
            border: 2px solid %3;
        }
        
        QComboBox::drop-down {
            subcontrol-origin: padding;
            subcontrol-position: top right;
            width: 24px;
            border-left: 1px solid %8;
            border-top-right-radius: 6px;
            border-bottom-right-radius: 6px;
            background-color: %2;
        }
        
        QComboBox::down-arrow {
            image: url(:/icons/down_arrow.png);
            width: 12px;
            height: 12px;
        }
        
        QComboBox::down-arrow:hover {
            image: url(:/icons/down_arrow_hover.png);
        }
        
        QComboBox QAbstractItemView {
            background-color: %1;
            color: %4;
            border: 1px solid %8;
            border-radius: 6px;
            selection-background-color: %3;
            selection-color: %1;
            outline: none;
        }
        
        /* ============================================================================
           슬라이더 - 세련된 범위 입력
        ============================================================================ */
        QSlider {
            background-color: transparent;
        }
        
        QSlider::groove:horizontal {
            border: 1px solid %8;
            height: 6px;
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %8, stop:1 %7);
            border-radius: 3px;
        }
        
        QSlider::handle:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %11, stop:1 %3);
            border: 2px solid %3;
            width: 16px;
            height: 16px;
            margin: -6px 0;
            border-radius: 10px;
        }
        
        QSlider::handle:horizontal:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %3, stop:1 %9);
            border: 2px solid %9;
        }
        
        QSlider::handle:horizontal:pressed {
            background-color: %9;
        }
        
        QSlider::sub-page:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %11, stop:1 %3);
            border-radius: 3px;
        }
        
        /* ============================================================================
           체크박스와 라디오 버튼 - 커스텀 인디케이터
        ============================================================================ */
        QCheckBox {
            color: %4;
            spacing: 8px;
            font-weight: 500;
        }
        
        QCheckBox::indicator {
            width: 16px;
            height: 16px;
            border: 2px solid %8;
            border-radius: 3px;
            background-color: %1;
        }
        
        QCheckBox::indicator:hover {
            border: 2px solid %3;
        }
        
        QCheckBox::indicator:checked {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %11, stop:1 %3);
            border: 2px solid %3;
            image: url(:/icons/check.png);
        }
        
        QCheckBox::indicator:checked:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %3, stop:1 %9);
        }
        
        QCheckBox::indicator:disabled {
            background-color: %8;
            border: 2px solid %8;
        }
        
        QRadioButton {
            color: %4;
            spacing: 8px;
            font-weight: 500;
        }
        
        QRadioButton::indicator {
            width: 16px;
            height: 16px;
            border: 2px solid %8;
            border-radius: 10px;
            background-color: %1;
        }
        
        QRadioButton::indicator:hover {
            border: 2px solid %3;
        }
        
        QRadioButton::indicator:checked {
            background: qradial-gradient(cx:0.5, cy:0.5, radius:0.5,
                                        stop:0 %3, stop:0.6 %3, stop:0.7 %1, stop:1 %1);
            border: 2px solid %3;
        }
        
        /* ============================================================================
           스핀박스와 라인에디트 - 입력 필드
        ============================================================================ */
        QLineEdit, QSpinBox, QDoubleSpinBox {
            background-color: %1;
            color: %4;
            border: 2px solid %8;
            border-radius: 6px;
            padding: 6px;
            selection-background-color: %3;
            selection-color: %1;
        }
        
        QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
            border: 2px solid %3;
        }
        
        QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {
            background-color: %8;
            color: %10;
        }
        
        QSpinBox::up-button, QDoubleSpinBox::up-button {
            subcontrol-origin: border;
            subcontrol-position: top right;
            width: 20px;
            border-left: 1px solid %8;
            border-bottom: 1px solid %8;
            border-top-right-radius: 6px;
            background-color: %2;
        }
        
        QSpinBox::down-button, QDoubleSpinBox::down-button {
            subcontrol-origin: border;
            subcontrol-position: bottom right;
            width: 20px;
            border-left: 1px solid %8;
            border-bottom-right-radius: 6px;
            background-color: %2;
        }
        
        QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
            image: url(:/icons/up_arrow.png);
            width: 8px;
            height: 8px;
        }
        
        QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
            image: url(:/icons/down_arrow.png);
            width: 8px;
            height: 8px;
        }
        
        /* ============================================================================
           스크롤바 - 세련된 스크롤 인터페이스
        ============================================================================ */
        QScrollBar:vertical {
            background-color: %7;
            width: 12px;
            border-radius: 6px;
            margin: 0;
        }
        
        QScrollBar::handle:vertical {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                       stop:0 %8, stop:1 %2);
            min-height: 20px;
            border-radius: 6px;
            margin: 2px;
        }
        
        QScrollBar::handle:vertical:hover {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                       stop:0 %3, stop:1 %11);
        }
        
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }
        
        QScrollBar:horizontal {
            background-color: %7;
            height: 12px;
            border-radius: 6px;
            margin: 0;
        }
        
        QScrollBar::handle:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %8, stop:1 %2);
            min-width: 20px;
            border-radius: 6px;
            margin: 2px;
        }
        
        QScrollBar::handle:horizontal:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %3, stop:1 %11);
        }
        
        /* ============================================================================
           라벨과 상태바 - 텍스트 표시
        ============================================================================ */
        QLabel {
            color: %4;
            background-color: transparent;
        }
        
        QLabel[accessibleName="title"] {
            font-size: 12pt;
            font-weight: 600;
            color: %3;
        }
        
        QLabel[accessibleName="subtitle"] {
            font-size: 10pt;
            font-weight: 500;
            color: %10;
        }
        
        QStatusBar {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %2, stop:1 %8);
            color: %4;
            border-top: 1px solid %8;
            padding: 4px;
            font-size: 8pt;
        }
        
        QStatusBar::item {
            border: none;
        }
        
        /* ============================================================================
           프로그레스바 - 진행 상태 표시
        ============================================================================ */
        QProgressBar {
            background-color: %7;
            border: 1px solid %8;
            border-radius: 6px;
            text-align: center;
            color: %4;
            font-weight: 600;
        }
        
        QProgressBar::chunk {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                       stop:0 %11, stop:1 %3);
            border-radius: 5px;
            margin: 1px;
        }
        
        /* ============================================================================
           툴팁 - 도움말 표시
        ============================================================================ */
        QToolTip {
            background-color: %2;
            color: %4;
            border: 1px solid %3;
            border-radius: 4px;
            padding: 4px 8px;
            font-size: 8pt;
        }
        
    )").arg(colors.primary)      // %1 - 주요 배경색
       .arg(colors.secondary)    // %2 - 보조 배경색  
       .arg(colors.accent)       // %3 - 강조색
       .arg(colors.text)         // %4 - 텍스트색
       .arg(primaryLight)        // %5 - 밝은 주색상 (사용하지 않음)
       .arg(primaryDark)         // %6 - 어두운 주색상 (사용하지 않음)
       .arg(secondaryLight)      // %7 - 밝은 보조색
       .arg(secondaryDark)       // %8 - 어두운 보조색
       .arg(accentDark)          // %9 - 어두운 강조색
       .arg(textMuted)           // %10 - 흐린 텍스트색
       .arg(accentLight);        // %11 - 밝은 강조색
}

} // namespace Theme