#ifndef THEME_MANAGER_HPP
#define THEME_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QHash>
#include <QColor>
#include <QApplication>
#include <ament_index_cpp/get_package_share_directory.hpp>  // 추가

namespace Theme {

struct ThemeColors {
    QString primary;      // color1 - 주요 배경색
    QString secondary;    // color2 - 보조 배경색
    QString accent;       // color3 - 강조색
    QString text;         // color4 - 텍스트색
};

struct ThemeData {
    QString name;
    ThemeColors colors;
};

class ThemeManager : public QObject {
    Q_OBJECT

public:
    static ThemeManager* instance();
    
    // 테마 관리
    bool loadThemesFromFile(const QString& filePath);
    QStringList getAvailableThemes() const;
    ThemeData getCurrentTheme() const;
    
    // 테마 적용
    void applyTheme(const QString& themeName);
    void applyDefaultTheme();

signals:
    void themeChanged(const QString& themeName);

private:
    explicit ThemeManager(QObject* parent = nullptr);
    
    // 스타일시트 생성
    QString generateStyleSheet(const ThemeColors& colors);
    
    // 기본 테마 초기화
    void initializeDefaultThemes();
    
    static ThemeManager* instance_;
    QHash<QString, ThemeData> themes_;
    QString currentThemeName_;
};

} // namespace Theme

#endif // THEME_MANAGER_HPP