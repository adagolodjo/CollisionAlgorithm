#pragma once

#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/config.h>

namespace sofa::collisionalgorithm {

class FixedProximity : public BaseProximity {
public:
    typedef std::shared_ptr<FixedProximity> SPtr;
    typedef typename collisionalgorithm::BaseProximity Inherits;
    typedef type::Vec3d Vec3d;

    FixedProximity(Vec3d P) : m_position(P) {
        m_normal = Vec3d();
    }
    FixedProximity(Vec3d P, Vec3d N) : m_position(P), m_normal(N) {
    }

    /// return proximity position in a Vec3
    Vec3d getPosition(core::VecCoordId = core::vec_id::write_access::position) const override {
        return m_position;
    }

    Vec3d getVelocity(core::VecDerivId = core::vec_id::write_access::velocity) const override {
        return Vec3d(0, 0, 0);
    }

    /// return normal in a Vec3
    Vec3d getNormal() const {
        return m_normal;
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId, const sofa::type::vector<sofa::type::Vec3>&, double,
                                 Index) const override {
    }

    void storeLambda(const core::ConstraintParams*, core::MultiVecDerivId, Index, Index,
                     const sofa::linearalgebra::BaseVector*) const override {
    }

    const std::type_info& getTypeInfo() const override {
        return typeid(FixedProximity);
    }

    ProximityType getProximityType() const override {
        return ProximityType::Fixed;
    }

    BaseProximity::SPtr copy() override {
        return SPtr(new FixedProximity(m_position, m_normal));
    }

    bool isNormalized() const override {
        return true;
    }

    void normalize() override {
    }

    double* getBaryCoord() override {
        return nullptr;
    }

private:
    Vec3d m_position;
    Vec3d m_normal;
};

}  // namespace sofa::collisionalgorithm
