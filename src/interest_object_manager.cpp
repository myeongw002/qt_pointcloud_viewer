#include "interest_object_manager.hpp"
#include <QDateTime>
#include <QDebug>

namespace ObjectManager {

InterestObjectManager& InterestObjectManager::instance() {
    static InterestObjectManager instance;
    return instance;
}

QString InterestObjectManager::registerInterestObject(
    Types::ObjectType type, 
    const QString& robotName, 
    const Types::Vec3& position) {
    
    QMutexLocker locker(&objectsMutex_);
    
    // 새 관심물체 생성
    QString objectId = generateObjectId();
    auto interestObject = std::make_shared<Types::InterestObject>(objectId, type, position, robotName);
    
    // 타입에 따른 기본 색상 설정
    switch (type) {
        case Types::ObjectType::OBSTACLE:
            interestObject->color = Types::ColorRGB(1.0f, 0.0f, 0.0f);  // 빨간색
            break;
        case Types::ObjectType::CUSTOM:
            interestObject->color = Types::ColorRGB(0.5f, 1.0f, 0.5f);  // 연녹색
            break;
        default:
            interestObject->color = Types::ColorRGB(1.0f, 1.0f, 1.0f);  // 흰색
            break;
    }
    
    // 설명 추가
    interestObject->description = QString("Discovered by %1 at %2")
        .arg(robotName)
        .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    
    // 저장
    interestObjects_[objectId] = interestObject;
    
    // 시그널 발생
    QString objectTypeStr = Types::objectTypeToString(type);
    emit interestObjectRegistered(objectId, objectTypeStr);
    
    qDebug() << "Global InterestObject registered:" << objectTypeStr
             << "at position:" << position.x << position.y << position.z
             << "by robot:" << robotName << "ID:" << objectId;
    
    return objectId;
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
        QString typeStr = Types::objectTypeToString(it.value()->type);
        result.append(QString("%1 (%2)").arg(typeStr, it.value()->id));
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
    return QString("OBJ_%1_%2").arg(++objectCounter_, 3, 10, QChar('0'));
}

} // namespace ObjectManager