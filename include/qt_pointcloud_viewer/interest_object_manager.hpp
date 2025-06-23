#pragma once

#include "common_types.hpp"
#include <QObject>
#include <QMutex>
#include <QHash>
#include <QStringList>

namespace ObjectManager {

class InterestObjectManager : public QObject {
    Q_OBJECT
    
public:
    // 싱글톤 인스턴스 접근
    static InterestObjectManager& instance();
    
    // 관심물체 등록/제거 (문자열 타입 사용)
    QString registerInterestObject(const QString& type, const QString& robotName, const Types::Vec3& position);
    
    // 지정된 ID로 객체 등록 (문자열 타입 사용)
    bool registerInterestObjectWithId(const QString& objectId, const QString& type, 
                                     const QString& robotName, const Types::Vec3& position);
    
    // 서비스 요청 데이터로 객체 등록 (문자열 타입 사용)
    bool registerInterestObjectFromService(const QString& serviceObjectId,
                                         const QString& objectType,
                                         const Types::Vec3& objectPosition,
                                         const QString& robotId);
    
    bool removeInterestObject(const QString& objectId);
    void clearAllInterestObjects();
    
    // 관심물체 조회
    Types::InterestObjectPtr getInterestObject(const QString& objectId) const;
    QStringList getAllObjectIds() const;
    QStringList getInterestObjectList() const;
    QHash<QString, Types::InterestObjectPtr> getAllInterestObjects() const;
    
    // 관심물체 속성 업데이트
    bool updateInterestObjectColor(const QString& objectId, const Types::ColorRGB& color);
    bool updateInterestObjectSize(const QString& objectId, float size);
    bool updateInterestObjectPosition(const QString& objectId, const Types::Vec3& position);
    
    // 색상 관련 함수 추가
    void reassignRandomColors();  // 모든 객체에 랜덤 색상 재할당
    
    // 표시 설정
    void setShowInterestObjects(bool show);
    bool getShowInterestObjects() const;

signals:
    void interestObjectRegistered(const QString& objectId, const QString& objectType);
    void interestObjectRemoved(const QString& objectId);
    void interestObjectUpdated(const QString& objectId);
    void showInterestObjectsChanged(bool show);

private:
    explicit InterestObjectManager(QObject* parent = nullptr);
    ~InterestObjectManager() = default;
    
    // 복사 생성자와 대입 연산자 삭제 (싱글톤)
    InterestObjectManager(const InterestObjectManager&) = delete;
    InterestObjectManager& operator=(const InterestObjectManager&) = delete;
    
    QString generateObjectId();
    
    mutable QMutex objectsMutex_;
    QHash<QString, Types::InterestObjectPtr> interestObjects_;
    int objectCounter_;
    bool showInterestObjects_;
};

} // namespace ObjectManager