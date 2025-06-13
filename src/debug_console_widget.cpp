#include "debug_console_widget.hpp"
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QScrollBar>
#include <QFont>
#include <QFontMetrics>
#include <QDebug>

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
    setupUI();
    setupMessageHandler();
    
    // 환영 메시지
    appendLog("🚀 Debug Console initialized", INFO);
    appendLog("📍 Ready to receive debug messages", INFO);
}

DebugConsoleWidget::~DebugConsoleWidget() {
    restoreMessageHandler();
    instance_ = nullptr;
}

void DebugConsoleWidget::setupUI() {
    setWindowTitle("Debug Console");
    resize(800, 400);
    
    // 메인 레이아웃
    auto* mainLayout = new QVBoxLayout(this);
    
    // 상단 컨트롤 패널
    auto* controlLayout = new QHBoxLayout();
    
    // 로그 레벨 선택
    auto* levelLabel = new QLabel("Log Level:");
    logLevelCombo_ = new QComboBox();
    logLevelCombo_->addItems({"Debug", "Info", "Warning", "Critical"});
    logLevelCombo_->setCurrentIndex(0);
    connect(logLevelCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DebugConsoleWidget::onLogLevelChanged);
    
    // 자동 스크롤 체크박스
    autoScrollCheckBox_ = new QCheckBox("Auto Scroll");
    autoScrollCheckBox_->setChecked(autoScroll_);
    connect(autoScrollCheckBox_, &QCheckBox::toggled,
            this, &DebugConsoleWidget::onAutoScrollToggled);
    
    // 단어 줄바꿈 체크박스
    wordWrapCheckBox_ = new QCheckBox("Word Wrap");
    wordWrapCheckBox_->setChecked(false);
    connect(wordWrapCheckBox_, &QCheckBox::toggled,
            this, &DebugConsoleWidget::onWordWrapToggled);
    
    // 버튼들
    clearButton_ = new QPushButton("Clear");
    saveButton_ = new QPushButton("Save to File");
    
    connect(clearButton_, &QPushButton::clicked, this, &DebugConsoleWidget::clearConsole);
    connect(saveButton_, &QPushButton::clicked, this, &DebugConsoleWidget::saveToFile);
    
    // 상태 라벨
    statusLabel_ = new QLabel("Ready");
    statusLabel_->setStyleSheet("color: green; font-weight: bold;");
    
    // 컨트롤 레이아웃에 추가
    controlLayout->addWidget(levelLabel);
    controlLayout->addWidget(logLevelCombo_);
    controlLayout->addWidget(autoScrollCheckBox_);
    controlLayout->addWidget(wordWrapCheckBox_);
    controlLayout->addStretch();
    controlLayout->addWidget(clearButton_);
    controlLayout->addWidget(saveButton_);
    controlLayout->addWidget(statusLabel_);
    
    // 콘솔 텍스트 에디터
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
    
    // 메인 레이아웃에 추가
    mainLayout->addLayout(controlLayout);
    mainLayout->addWidget(consoleTextEdit_);
    
    // 마진 설정
    mainLayout->setContentsMargins(5, 5, 5, 5);
    controlLayout->setContentsMargins(0, 0, 0, 0);
}

void DebugConsoleWidget::setupMessageHandler() {
    // 이전 핸들러 저장
    previousHandler_ = qInstallMessageHandler(nullptr);
    
    // 새로운 핸들러 설치
    qInstallMessageHandler([](QtMsgType type, const QMessageLogContext& context, const QString& msg) {
        // 이전 핸들러가 있으면 호출
        if (DebugConsoleWidget::instance_ && DebugConsoleWidget::instance_->previousHandler_) {
            DebugConsoleWidget::instance_->previousHandler_(type, context, msg);
        }
        
        // 콘솔에 메시지 추가
        if (DebugConsoleWidget::instance_) {
            LogLevel level = INFO;
            switch (type) {
                case QtDebugMsg:    level = DEBUG; break;
                case QtInfoMsg:     level = INFO; break;
                case QtWarningMsg:  level = WARNING; break;
                case QtCriticalMsg:
                case QtFatalMsg:    level = CRITICAL; break;
            }
            
            // GUI 스레드에서 실행되도록 큐에 추가
            QMetaObject::invokeMethod(DebugConsoleWidget::instance_, 
                                    "appendLog", 
                                    Qt::QueuedConnection,
                                    Q_ARG(QString, msg),
                                    Q_ARG(LogLevel, level));
        }
    });
}

void DebugConsoleWidget::restoreMessageHandler() {
    qInstallMessageHandler(previousHandler_);
}

void DebugConsoleWidget::appendLog(const QString& message, LogLevel level) {
    // 로그 레벨 필터링
    if (level < currentLogLevel_) {
        return;
    }
    
    // 최대 라인 수 체크
    if (consoleTextEdit_->document()->blockCount() > maxLogLines_) {
        // 처음 100줄 삭제
        QTextCursor cursor = consoleTextEdit_->textCursor();
        cursor.movePosition(QTextCursor::Start);
        for (int i = 0; i < 100; ++i) {
            cursor.select(QTextCursor::BlockUnderCursor);
            cursor.removeSelectedText();
            cursor.deleteChar(); // 줄바꿈 문자 삭제
        }
    }
    
    // 포맷된 메시지 생성
    QString formattedMessage = formatLogMessage(message, level);
    
    // HTML로 색상 적용
    QColor color = getLogLevelColor(level);
    QString htmlMessage = QString("<span style='color: %1;'>%2</span>")
                          .arg(color.name())
                          .arg(formattedMessage.toHtmlEscaped());
    
    // 텍스트 추가
    consoleTextEdit_->append(htmlMessage);
    
    // 자동 스크롤
    if (autoScroll_) {
        QScrollBar* scrollBar = consoleTextEdit_->verticalScrollBar();
        scrollBar->setValue(scrollBar->maximum());
    }
    
    // 상태 업데이트
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
        case DEBUG:    return QColor("#888888");  // 회색
        case INFO:     return QColor("#ffffff");  // 흰색
        case WARNING:  return QColor("#ffaa00");  // 주황색
        case CRITICAL: return QColor("#ff4444");  // 빨간색
        default:       return QColor("#ffffff");
    }
}

void DebugConsoleWidget::clearConsole() {
    consoleTextEdit_->clear();
    statusLabel_->setText("Cleared");
    appendLog("🧹 Console cleared", INFO);
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
            appendLog(QString("💾 Log saved to: %1").arg(fileName), INFO);
        } else {
            QMessageBox::warning(this, "Error", "Could not save file!");
        }
    }
}

void DebugConsoleWidget::onAutoScrollToggled(bool enabled) {
    autoScroll_ = enabled;
    appendLog(QString("🔄 Auto scroll: %1").arg(enabled ? "ON" : "OFF"), DEBUG);
}

void DebugConsoleWidget::onLogLevelChanged(int level) {
    currentLogLevel_ = static_cast<LogLevel>(level);
    QString levelName = logLevelCombo_->currentText();
    appendLog(QString("📊 Log level changed to: %1").arg(levelName), DEBUG);
}

void DebugConsoleWidget::onWordWrapToggled(bool enabled) {
    consoleTextEdit_->setLineWrapMode(enabled ? QTextEdit::WidgetWidth : QTextEdit::NoWrap);
    appendLog(QString("📝 Word wrap: %1").arg(enabled ? "ON" : "OFF"), DEBUG);
}

} // namespace Widget

#include "debug_console_widget.moc"