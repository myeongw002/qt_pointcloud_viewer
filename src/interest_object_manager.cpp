#include "interest_object_manager.hpp"
#include <QDateTime>
#include <QDebug>
#include <QMutexLocker>

namespace ObjectManager {

InterestObjectManager::InterestObjectManager(QObject* parent)
    : QObject(parent)
    , objectCounter_(0)
    , showInterestObjects_(true) {
}

InterestObjectManager& InterestObjectManager::instance() {
    static InterestObjectManager instance;
    return instance;
}

QString InterestObjectManager::registerInterestObject(
    const QString& type, 
    const QString& robotName, 
    const Types::Vec3& position) {
    
    QMutexLocker locker(&objectsMutex_);
    
    // 새 관심물체 생성
    QString objectId = generateObjectId();
    auto interestObject = std::make_shared<Types::InterestObject>(objectId, type, position, robotName);
    
    // 저장
    interestObjects_[objectId] = interestObject;
    
    // 시그널 발생
    emit interestObjectRegistered(objectId, type);
    
    qDebug() << "Global InterestObject registered:" << type
             << "at position:" << position.x << position.y << position.z
             << "by robot:" << robotName << "ID:" << objectId
             << "Color: RGB(" << interestObject->color.x << "," 
             << interestObject->color.y << "," << interestObject->color.z << ")";
    
    return objectId;
}

bool InterestObjectManager::registerInterestObjectWithId(
    const QString& objectId,
    const QString& type, 
    const QString& robotName, 
    const Types::Vec3& position) {
    
    QMutexLocker locker(&objectsMutex_);
    
    // 이미 존재하는 ID인지 확인
    if (interestObjects_.contains(objectId)) {
        qDebug() << "Object with ID" << objectId << "already exists, updating position";
        
        // 기존 객체 업데이트 (색상은 유지)
        auto& existingObj = interestObjects_[objectId];
        existingObj->position = position;
        existingObj->type = type;
        existingObj->discoveredBy = robotName;
        existingObj->lastUpdateTime = QDateTime::currentDateTime();
        existingObj->timestamp = std::chrono::steady_clock::now();
        // 색상은 기존 것을 유지 (변경하지 않음)
        
        // 업데이트 시그널 발생
        emit interestObjectUpdated(objectId);
        return false;  // 새 등록이 아닌 업데이트
    }
    
    // 새 객체 생성
    auto interestObject = std::make_shared<Types::InterestObject>(objectId, type, position, robotName);
    
    // 저장
    interestObjects_[objectId] = interestObject;
    
    qDebug() << "Registered new Interest Object:" << objectId 
             << "Type:" << type
             << "Robot:" << robotName
             << "Position:" << position.x << position.y << position.z
             << "Color: RGB(" << interestObject->color.x << "," 
             << interestObject->color.y << "," << interestObject->color.z << ")";
    
    // 시그널 발생
    emit interestObjectRegistered(objectId, type);
    
    return true;
}

// 서비스 요청 데이터로 객체 등록 (랜덤 색상 적용)
bool InterestObjectManager::registerInterestObjectFromService(
    const QString& serviceObjectId,
    const QString& objectType,
    const Types::Vec3& objectPosition,
    const QString& robotId) {
    
    QMutexLocker locker(&objectsMutex_);
    
    // 서비스 Object ID를 키로 사용
    QString fullObjectId = serviceObjectId;
    
    // 이미 존재하는 객체인지 확인
    if (interestObjects_.contains(fullObjectId)) {
        qDebug() << "Object with service ID" << serviceObjectId << "already exists, updating...";
        
        // 기존 객체 업데이트 (색상은 유지)
        auto& existingObj = interestObjects_[fullObjectId];
        existingObj->position = objectPosition;
        existingObj->type = objectType;
        existingObj->discoveredBy = robotId;
        existingObj->lastUpdateTime = QDateTime::currentDateTime();
        existingObj->timestamp = std::chrono::steady_clock::now();
        // 색상은 기존 것을 유지 (변경하지 않음)
        
        // 설명 업데이트
        existingObj->description = QString("Object #%1 (%2) discovered by %3, updated at %4")
            .arg(serviceObjectId)
            .arg(objectType)
            .arg(robotId)
            .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
        
        // 업데이트 시그널 발생
        emit interestObjectUpdated(fullObjectId);
        return false;  // 새 등록이 아닌 업데이트
    }
    
    // 새 객체 생성 (랜덤 색상으로)
    auto interestObject = std::make_shared<Types::InterestObject>(
        serviceObjectId,    // 서비스에서 온 Object ID
        objectType,         // 서비스에서 온 Object Class (문자열 그대로)
        objectPosition,     // 계산된 Position  
        robotId             // 서비스에서 온 Robot ID
    );
    
    // 저장
    interestObjects_[fullObjectId] = interestObject;
    
    qDebug() << "🎨 Registered new Interest Object from service:" 
             << "ID:" << serviceObjectId
             << "Type:" << objectType
             << "Robot:" << robotId
             << "Position:" << objectPosition.x << objectPosition.y << objectPosition.z
             << "Random Color: RGB(" << interestObject->color.x << "," 
             << interestObject->color.y << "," << interestObject->color.z << ")";
    
    // 시그널 발생
    emit interestObjectRegistered(fullObjectId, objectType);
    
    return true;  // 새 등록 성공
}

// 색상 강제 재할당 함수 추가
void InterestObjectManager::reassignRandomColors() {
    QMutexLocker locker(&objectsMutex_);
    
    for (auto it = interestObjects_.begin(); it != interestObjects_.end(); ++it) {
        auto& obj = it.value();
        obj->color = Types::InterestObjectColors::getRandomColor();
        emit interestObjectUpdated(it.key());
    }
    
    qDebug() << "🎨 Reassigned random colors to all" << interestObjects_.size() << "interest objects";
}

bool InterestObjectManager::removeInterestObject(const QString& objectId) {
    QMutexLocker locker(&objectsMutex_);
    
    auto it = interestObjects_.find(objectId);
    if (it != interestObjects_.end()) {
        interestObjects_.erase(it);
        emit interestObjectRemoved(objectId);
        qDebug() << "Global InterestObject removed, ID:" << objectId;
        return true;
    }
    
    return false;
}

void InterestObjectManager::clearAllInterestObjects() {
    QStringList objectIds;
    
    {
        QMutexLocker locker(&objectsMutex_);
        for (auto it = interestObjects_.cbegin(); it != interestObjects_.cend(); ++it) {
            objectIds.append(it.key());
        }
        interestObjects_.clear();
    }
    
    // 각 객체에 대해 제거 시그널 발생
    for (const QString& objectId : objectIds) {
        emit interestObjectRemoved(objectId);
    }
    
    qDebug() << "All global InterestObjects cleared, count:" << objectIds.size();
}

Types::InterestObjectPtr InterestObjectManager::getInterestObject(const QString& objectId) const {
    QMutexLocker locker(&objectsMutex_);
    auto it = interestObjects_.find(objectId);
    return (it != interestObjects_.end()) ? it.value() : nullptr;
}

QStringList InterestObjectManager::getAllObjectIds() const {
    QMutexLocker locker(&objectsMutex_);
    return interestObjects_.keys();
}

QStringList InterestObjectManager::getInterestObjectList() const {
    QMutexLocker locker(&objectsMutex_);
    QStringList result;
    
    for (auto it = interestObjects_.cbegin(); it != interestObjects_.cend(); ++it) {
        result.append(QString("%1 (%2)").arg(it.value()->type, it.value()->id));
    }
    
    return result;
}

QHash<QString, Types::InterestObjectPtr> InterestObjectManager::getAllInterestObjects() const {
    QMutexLocker locker(&objectsMutex_);
    return interestObjects_;  // 복사본 반환
}

bool InterestObjectManager::updateInterestObjectColor(const QString& objectId, const Types::ColorRGB& color) {
    QMutexLocker locker(&objectsMutex_);
    auto it = interestObjects_.find(objectId);
    if (it != interestObjects_.end()) {
        it.value()->color = color;
        emit interestObjectUpdated(objectId);
        qDebug() << "Updated color for object" << objectId 
                 << "to RGB(" << color.x << "," << color.y << "," << color.z << ")";
        return true;
    }
    return false;
}

bool InterestObjectManager::updateInterestObjectSize(const QString& objectId, float size) {
    QMutexLocker locker(&objectsMutex_);
    auto it = interestObjects_.find(objectId);
    if (it != interestObjects_.end()) {
        it.value()->size = size;
        emit interestObjectUpdated(objectId);
        return true;
    }
    return false;
}

bool InterestObjectManager::updateInterestObjectPosition(const QString& objectId, const Types::Vec3& position) {
    QMutexLocker locker(&objectsMutex_);
    auto it = interestObjects_.find(objectId);
    if (it != interestObjects_.end()) {
        it.value()->position = position;
        emit interestObjectUpdated(objectId);
        return true;
    }
    return false;
}

void InterestObjectManager::setShowInterestObjects(bool show) {
    if (showInterestObjects_ != show) {
        showInterestObjects_ = show;
        emit showInterestObjectsChanged(show);
    }
}

bool InterestObjectManager::getShowInterestObjects() const {
    return showInterestObjects_;
}

QString InterestObjectManager::generateObjectId() {
    return QString("OBJ_%1_%2").arg(QDateTime::currentMSecsSinceEpoch()).arg(++objectCounter_, 3, 10, QChar('0'));
}

} // namespace ObjectManager