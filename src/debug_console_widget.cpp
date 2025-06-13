#include "debug_console_widget.hpp"
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QScrollBar>
#include <QFont>
#include <QFontMetrics>
#include <QDebug>
#include <QThread>

namespace Widget {

DebugConsoleWidget* DebugConsoleWidget::instance_ = nullptr;

DebugConsoleWidget::DebugConsoleWidget(QWidget* parent)
    : QWidget(parent)
    , consoleTextEdit_(nullptr)
    , clearButton_(nullptr)
    , saveButton_(nullptr)
    , autoScrollCheckBox_(nullptr)
    , wordWrapCheckBox_(nullptr)
    , logLevelCombo_(nullptr)
    , statusLabel_(nullptr)
    , autoScroll_(true)
    , currentLogLevel_(DEBUG)
    , maxLogLines_(1000)
    , previousHandler_(nullptr)
{
    instance_ = this;
    
    // Register LogLevel metatype for Qt's meta system
    qRegisterMetaType<Widget::DebugConsoleWidget::LogLevel>("LogLevel");
    qRegisterMetaType<Widget::DebugConsoleWidget::LogLevel>("Widget::DebugConsoleWidget::LogLevel");
    
    setupUI();
    setupMessageHandler();
    
    // Welcome messages
    appendLog("Debug Console initialized", INFO);
    appendLog("Ready to receive debug messages", INFO);
}

DebugConsoleWidget::~DebugConsoleWidget() {
    restoreMessageHandler();
    instance_ = nullptr;
}

void DebugConsoleWidget::setupUI() {
    setWindowTitle("Debug Console");
    resize(800, 400);
    
    // Main layout
    auto* mainLayout = new QVBoxLayout(this);
    
    // Top control panel
    auto* controlLayout = new QHBoxLayout();
    
    // Log level selection
    auto* levelLabel = new QLabel("Log Level:");
    logLevelCombo_ = new QComboBox();
    logLevelCombo_->addItems({"Debug", "Info", "Warning", "Critical"});
    logLevelCombo_->setCurrentIndex(0);
    connect(logLevelCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DebugConsoleWidget::onLogLevelChanged);
    
    // Auto scroll checkbox
    autoScrollCheckBox_ = new QCheckBox("Auto Scroll");
    autoScrollCheckBox_->setChecked(autoScroll_);
    connect(autoScrollCheckBox_, &QCheckBox::toggled,
            this, &DebugConsoleWidget::onAutoScrollToggled);
    
    // Word wrap checkbox
    wordWrapCheckBox_ = new QCheckBox("Word Wrap");
    wordWrapCheckBox_->setChecked(false);
    connect(wordWrapCheckBox_, &QCheckBox::toggled,
            this, &DebugConsoleWidget::onWordWrapToggled);
    
    // Buttons
    clearButton_ = new QPushButton("Clear");
    saveButton_ = new QPushButton("Save to File");
    
    connect(clearButton_, &QPushButton::clicked, this, &DebugConsoleWidget::clearConsole);
    connect(saveButton_, &QPushButton::clicked, this, &DebugConsoleWidget::saveToFile);
    
    // Status label
    statusLabel_ = new QLabel("Ready");
    statusLabel_->setStyleSheet("color: green; font-weight: bold;");
    
    // Add to control layout
    controlLayout->addWidget(levelLabel);
    controlLayout->addWidget(logLevelCombo_);
    controlLayout->addWidget(autoScrollCheckBox_);
    controlLayout->addWidget(wordWrapCheckBox_);
    controlLayout->addStretch();
    controlLayout->addWidget(clearButton_);
    controlLayout->addWidget(saveButton_);
    controlLayout->addWidget(statusLabel_);
    
    // Console text editor
    consoleTextEdit_ = new QTextEdit();
    consoleTextEdit_->setReadOnly(true);
    consoleTextEdit_->setFont(QFont("Consolas", 9));
    consoleTextEdit_->setStyleSheet(
        "QTextEdit {"
        "  background-color: #1e1e1e;"
        "  color: #ffffff;"
        "  border: 1px solid #555;"
        "  selection-background-color: #444;"
        "}"
    );
    
    // Add to main layout
    mainLayout->addLayout(controlLayout);
    mainLayout->addWidget(consoleTextEdit_);
    
    // Set margins
    mainLayout->setContentsMargins(5, 5, 5, 5);
    controlLayout->setContentsMargins(0, 0, 0, 0);
}

void DebugConsoleWidget::setupMessageHandler() {
    // Store previous handler
    previousHandler_ = qInstallMessageHandler(nullptr);
    
    // Install new handler
    qInstallMessageHandler([](QtMsgType type, const QMessageLogContext& context, const QString& msg) {
        // Call previous handler if exists
        if (DebugConsoleWidget::instance_ && DebugConsoleWidget::instance_->previousHandler_) {
            DebugConsoleWidget::instance_->previousHandler_(type, context, msg);
        }
        
        // Add message to console
        if (DebugConsoleWidget::instance_) {
            LogLevel level = INFO;
            switch (type) {
                case QtDebugMsg:    level = DEBUG; break;
                case QtInfoMsg:     level = INFO; break;
                case QtWarningMsg:  level = WARNING; break;
                case QtCriticalMsg:
                case QtFatalMsg:    level = CRITICAL; break;
            }
            
            // Use direct call instead of QMetaObject::invokeMethod to avoid type registration issues
            // Check if we're in the GUI thread
            if (QThread::currentThread() == QApplication::instance()->thread()) {
                // Direct call if in GUI thread
                DebugConsoleWidget::instance_->appendLog(msg, level);
            } else {
                // Use queued connection for cross-thread calls
                QMetaObject::invokeMethod(DebugConsoleWidget::instance_, 
                                        [=]() {
                                            DebugConsoleWidget::instance_->appendLog(msg, level);
                                        }, 
                                        Qt::QueuedConnection);
            }
        }
    });
}

void DebugConsoleWidget::restoreMessageHandler() {
    qInstallMessageHandler(previousHandler_);
}

void DebugConsoleWidget::appendLog(const QString& message, LogLevel level) {
    // Filter by log level
    if (level < currentLogLevel_) {
        return;
    }
    
    // Check maximum line count
    if (consoleTextEdit_->document()->blockCount() > maxLogLines_) {
        // Remove first 100 lines
        QTextCursor cursor = consoleTextEdit_->textCursor();
        cursor.movePosition(QTextCursor::Start);
        for (int i = 0; i < 100; ++i) {
            cursor.select(QTextCursor::BlockUnderCursor);
            cursor.removeSelectedText();
            cursor.deleteChar(); // Remove newline character
        }
    }
    
    // Generate formatted message
    QString formattedMessage = formatLogMessage(message, level);
    
    // Apply color with HTML
    QColor color = getLogLevelColor(level);
    QString htmlMessage = QString("<span style='color: %1;'>%2</span>")
                          .arg(color.name())
                          .arg(formattedMessage.toHtmlEscaped());
    
    // Add text
    consoleTextEdit_->append(htmlMessage);
    
    // Auto scroll
    if (autoScroll_) {
        QScrollBar* scrollBar = consoleTextEdit_->verticalScrollBar();
        scrollBar->setValue(scrollBar->maximum());
    }
    
    // Update status
    int totalLines = consoleTextEdit_->document()->blockCount();
    statusLabel_->setText(QString("Lines: %1").arg(totalLines));
}

QString DebugConsoleWidget::formatLogMessage(const QString& message, LogLevel level) {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString levelStr;
    
    switch (level) {
        case DEBUG:    levelStr = "DEBUG"; break;
        case INFO:     levelStr = "INFO "; break;
        case WARNING:  levelStr = "WARN "; break;
        case CRITICAL: levelStr = "ERROR"; break;
    }
    
    return QString("[%1] [%2] %3").arg(timestamp, levelStr, message);
}

QColor DebugConsoleWidget::getLogLevelColor(LogLevel level) {
    switch (level) {
        case DEBUG:    return QColor("#888888");  // Gray
        case INFO:     return QColor("#ffffff");  // White
        case WARNING:  return QColor("#ffaa00");  // Orange
        case CRITICAL: return QColor("#ff4444");  // Red
        default:       return QColor("#ffffff");
    }
}

void DebugConsoleWidget::clearConsole() {
    consoleTextEdit_->clear();
    statusLabel_->setText("Cleared");
    appendLog("Console cleared", INFO);
}

void DebugConsoleWidget::saveToFile() {
    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save Debug Log",
        QString("debug_log_%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")),
        "Text Files (*.txt);;All Files (*)"
    );
    
    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream stream(&file);
            stream << consoleTextEdit_->toPlainText();
            file.close();
            
            statusLabel_->setText("Saved to file");
            appendLog(QString("Log saved to: %1").arg(fileName), INFO);
        } else {
            QMessageBox::warning(this, "Error", "Could not save file!");
        }
    }
}

void DebugConsoleWidget::onAutoScrollToggled(bool enabled) {
    autoScroll_ = enabled;
    appendLog(QString("Auto scroll: %1").arg(enabled ? "ON" : "OFF"), DEBUG);
}

void DebugConsoleWidget::onLogLevelChanged(int level) {
    currentLogLevel_ = static_cast<LogLevel>(level);
    QString levelName = logLevelCombo_->currentText();
    appendLog(QString("Log level changed to: %1").arg(levelName), DEBUG);
}

void DebugConsoleWidget::onWordWrapToggled(bool enabled) {
    consoleTextEdit_->setLineWrapMode(enabled ? QTextEdit::WidgetWidth : QTextEdit::NoWrap);
    appendLog(QString("Word wrap: %1").arg(enabled ? "ON" : "OFF"), DEBUG);
}

} // namespace Widget

#include "debug_console_widget.moc"