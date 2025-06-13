#ifndef DEBUG_CONSOLE_WIDGET_H
#define DEBUG_CONSOLE_WIDGET_H

#include <QWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QTimer>
#include <QScrollBar>
#include <QDateTime>
#include <QLoggingCategory>

namespace Widget {

class DebugConsoleWidget : public QWidget {
    Q_OBJECT

public:
    explicit DebugConsoleWidget(QWidget* parent = nullptr);
    ~DebugConsoleWidget();

    // Log level enumeration
    enum LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARNING = 2,
        CRITICAL = 3
    };
    Q_ENUM(LogLevel)  // Register enum with Qt meta system

public slots:
    void appendLog(const QString& message, LogLevel level = INFO);
    void clearConsole();
    void saveToFile();

private slots:
    void onAutoScrollToggled(bool enabled);
    void onLogLevelChanged(int level);
    void onWordWrapToggled(bool enabled);

private:
    void setupUI();
    void setupMessageHandler();
    void restoreMessageHandler();
    QString formatLogMessage(const QString& message, LogLevel level);
    QColor getLogLevelColor(LogLevel level);

private:
    // UI components
    QTextEdit* consoleTextEdit_;
    QPushButton* clearButton_;
    QPushButton* saveButton_;
    QCheckBox* autoScrollCheckBox_;
    QCheckBox* wordWrapCheckBox_;
    QComboBox* logLevelCombo_;
    QLabel* statusLabel_;

    // Settings
    bool autoScroll_;
    LogLevel currentLogLevel_;
    int maxLogLines_;

    // Qt message handler related
    static DebugConsoleWidget* instance_;
    QtMessageHandler previousHandler_;
};

} // namespace Widget

// Register LogLevel for use with QMetaObject::invokeMethod
Q_DECLARE_METATYPE(Widget::DebugConsoleWidget::LogLevel)

#endif // DEBUG_CONSOLE_WIDGET_H