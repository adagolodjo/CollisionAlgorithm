#include <CollisionAlgorithm/BaseElement.h>
#include <CollisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionalgorithm {

type::BoundingBox BaseElement::getBoundingBox() const {
    type::BoundingBox bbox;

    const ElementContainer<PointElement>& points = pointElements();
    for (auto it = points.cbegin(); it != points.cend(); ++it) {
        const BaseProximity::SPtr& prox = (*it)->getP0();
        if (prox != nullptr) bbox.include(prox->getPosition());
    }

    return bbox;
}

}  // namespace sofa::collisionalgorithm
