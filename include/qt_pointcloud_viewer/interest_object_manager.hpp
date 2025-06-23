#ifndef INTEREST_OBJECT_MANAGER_HPP
#define INTEREST_OBJECT_MANAGER_HPP

#include <QObject>
#include <QHash>
#include <QStringList>
#include <QMutex>
#include <QMutexLocker>
#include "common_types.hpp"

namespace ObjectManager {

class InterestObjectManager : public QObject {
    Q_OBJECT

public:
    static InterestObjectManager& instance();
    
    // Interest Object 관리
    QString registerInterestObject(Types::ObjectType type, const QString& robotName, const Types::Vec3& position);
    bool removeInterestObject(const QString& objectId);
    void clearAllInterestObjects();
    
    // 조회 함수들
    Types::InterestObjectPtr getInterestObject(const QString& objectId) const;
    QStringList getAllObjectIds() const;
    QStringList getInterestObjectList() const;
    QHash<QString, Types::InterestObjectPtr> getAllInterestObjects() const;
    
    // 설정 함수들
    bool updateInterestObjectColor(const QString& objectId, const Types::ColorRGB& color);
    bool updateInterestObjectSize(const QString& objectId, float size);
    bool updateInterestObjectPosition(const QString& objectId, const Types::Vec3& position);
    
    // 상태 관리
    void setShowInterestObjects(bool show);
    bool getShowInterestObjects() const;

signals:
    void interestObjectRegistered(const QString& objectId, const QString& objectType);
    void interestObjectRemoved(const QString& objectId);
    void interestObjectUpdated(const QString& objectId);
    void showInterestObjectsChanged(bool show);

private:
    InterestObjectManager() = default;
    ~InterestObjectManager() = default;
    
    // 싱글톤 패턴
    InterestObjectManager(const InterestObjectManager&) = delete;
    InterestObjectManager& operator=(const InterestObjectManager&) = delete;
    
    // 데이터 저장소
    QHash<QString, Types::InterestObjectPtr> interestObjects_;
    mutable QMutex objectsMutex_;
    
    // 설정
    bool showInterestObjects_ = true;
    int objectCounter_ = 0;
    
    // 헬퍼 함수
    QString generateObjectId();
};

} // namespace ObjectManager

#endif // INTEREST_OBJECT_MANAGER_HPP