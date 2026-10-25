#include <CollisionAlgorithm/proximity/PointProximity.h>
#include <CollisionAlgorithm/toolbox/PointToolBox.h>

namespace sofa::collisionalgorithm::toolbox {

Operations::CreateCenterProximity::Result PointToolBox::createCenterProximity(const PointElement::SPtr& point) {
    return point->getP0();
}

Operations::Project::Result PointToolBox::project(const type::Vec3& P, const PointElement::SPtr& point) {
    double dist = (P - point->getP0()->getPosition()).norm();
    BaseProximity::SPtr prox = point->getP0();

    return Operations::Project::Result(dist, prox);
}

}  // namespace sofa::collisionalgorithm::toolbox
