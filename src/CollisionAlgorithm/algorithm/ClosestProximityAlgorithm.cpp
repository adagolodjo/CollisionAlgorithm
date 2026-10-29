#include <CollisionAlgorithm/algorithm/ClosestProximityAlgorithm.h>
#include <CollisionAlgorithm/filters/DistanceFilter.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>
#include <CollisionAlgorithm/operations/FindClosestProximity.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/core/ObjectFactory.h>

#include <algorithm>
#include <limits>

namespace sofa::collisionalgorithm {

void registerClosestProximityAlgorithm(sofa::core::ObjectFactory* factory) {
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "Closest proximity on a surface for samples of the elements of a geometry (contact of a rod with a surface)")
            .add<ClosestProximityAlgorithm>());
}

ClosestProximityAlgorithm::ClosestProximityAlgorithm()
    : l_from(initLink("from", "Link to the geometry whose elements are sampled (rod, shaft...).")),
      l_dest(initLink("dest", "Link to the geometry on which the closest proximities are searched (surface).")),
      d_output(initData(&d_output, "collisionOutput", "Pairs (sample, closest proximity) closer than 'distance'.")),
      d_samples(initData(&d_samples, 1, "samples", "Samples per edge of 'from' (at the middle of equal parts).")),
      d_projective(initData(&d_projective, true, "projective",
                            "One pair per edge, at the point of the edge closest to the surface (ignores 'samples').")),
      d_projectiveIterations(initData(&d_projectiveIterations, 8, "projectiveIterations",
                                      "Golden-section refinements of the position of the closest approach along the edge (with 'projective').")),
      d_distance(initData(&d_distance, std::numeric_limits<SReal>::max(), "distance",
                          "Farthest distance between the two proximities of a pair.")),
      d_drawCollision(initData(&d_drawCollision, false, "drawcollision", "Draw the pairs.")) {
}

double ClosestProximityAlgorithm::getMaxDistance() const {
    double maxDistance = d_distance.getValue();
    for (const auto& filter : m_filters) {
        if (const auto* distanceFilter = dynamic_cast<const DistanceFilter*>(filter.get()))
            maxDistance = std::min(maxDistance, distanceFilter->d_distance.getValue());
    }
    return maxDistance;
}

void ClosestProximityAlgorithm::doDetection() {
    auto& output = *d_output.beginEdit();
    output.clear();

    if (l_from == nullptr || l_dest == nullptr) {
        d_output.endEdit();
        return;
    }

    ElementIterator::SPtr itFrom = l_from->begin();
    if (itFrom->end()) {
        d_output.endEdit();
        return;
    }

    auto createCenterProximity = Operations::CreateCenterProximity::Operation::get(itFrom->getTypeInfo());
    auto findClosest = Operations::FindClosestProximity::Operation::get(l_dest);
    auto projectOnDest = Operations::Project::Operation::get(l_dest);
    const FilterFUNC filter = getFilterFunc();
    const double maxDistance = getMaxDistance();
    const int samples = std::max(1, d_samples.getValue());
    const bool projective = d_projective.getValue();
    const bool edges = itFrom->getTypeInfo() == typeid(EdgeElement);

    // closest proximity of `dest` to a sample, and the distance to it (infinity if none within reach)
    auto query = [&](const BaseProximity::SPtr& sample, double reach, BaseProximity::SPtr& closest) {
        closest = findClosest(sample, l_dest.get(), projectOnDest, filter, reach);
        if (closest == nullptr) return std::numeric_limits<double>::max();
        closest->normalize();
        return (sample->getPosition() - closest->getPosition()).norm();
    };

    for (; !itFrom->end(); itFrom++) {
        const BaseElement::SPtr element = itFrom->element();
        if (element == nullptr) continue;

        if (edges && projective) {
            // One pair per edge, at the closest approach of the edge to the surface: the distance to the surface along the
            // edge is evaluated at both ends and in the middle, then refined by a golden-section search in the best bracket.
            const auto edge = std::static_pointer_cast<EdgeElement>(element);
            const double halfLength = 0.5 * (edge->getP1()->getPosition() - edge->getP0()->getPosition()).norm();

            struct Eval {
                double t, d;
                BaseProximity::SPtr sample, closest;
            };
            auto evaluate = [&](double t, double reach) {
                Eval e{t, 0.0, EdgeProximity::create(edge, 1.0 - t, t), nullptr};
                e.d = query(e.sample, reach, e.closest);
                return e;
            };

            Eval mid = evaluate(0.5, maxDistance + halfLength);  // the edge cannot come closer than this to the surface
            if (mid.closest == nullptr) continue;
            Eval best = mid;
            Eval left = evaluate(0.0, maxDistance), right = evaluate(1.0, maxDistance);
            for (const Eval* e : {&left, &right})
                if (e->d < best.d) best = *e;

            // bracket [lo, hi] around the best of the three evaluations
            double lo = (best.t == 0.0) ? 0.0 : (best.t == 0.5 ? 0.0 : 0.5);
            double hi = (best.t == 1.0) ? 1.0 : (best.t == 0.5 ? 1.0 : 0.5);
            const double inv = 0.6180339887498949;
            double x1 = hi - inv * (hi - lo), x2 = lo + inv * (hi - lo);
            Eval e1 = evaluate(x1, maxDistance), e2 = evaluate(x2, maxDistance);
            for (int i = 0; i < d_projectiveIterations.getValue(); i++) {
                if (e1.d < best.d) best = e1;
                if (e2.d < best.d) best = e2;
                if (e1.d <= e2.d) {
                    hi = x2;
                    x2 = x1;
                    e2 = e1;
                    x1 = hi - inv * (hi - lo);
                    e1 = evaluate(x1, maxDistance);
                } else {
                    lo = x1;
                    x1 = x2;
                    e1 = e2;
                    x2 = lo + inv * (hi - lo);
                    e2 = evaluate(x2, maxDistance);
                }
            }
            if (e1.d < best.d) best = e1;
            if (e2.d < best.d) best = e2;

            if (best.closest == nullptr || best.d >= maxDistance || !filter(best.sample, best.closest)) continue;
            output.add(best.sample, best.closest);
            continue;
        }

        const int count = edges ? samples : 1;
        for (int s = 0; s < count; s++) {
            BaseProximity::SPtr sample;
            if (edges && count > 1) {
                const double t = (s + 0.5) / count;
                sample = EdgeProximity::create(std::static_pointer_cast<EdgeElement>(element), 1.0 - t, t);
            } else {
                sample = createCenterProximity(element);
            }
            if (sample == nullptr) continue;

            BaseProximity::SPtr closest;
            const double d = query(sample, maxDistance, closest);
            if (closest == nullptr || d >= maxDistance) continue;
            output.add(sample, closest);
        }
    }

    d_output.endEdit();
}

void ClosestProximityAlgorithm::draw(const core::visual::VisualParams* vparams) {
    if (!vparams->displayFlags().getShowCollisionModels() && !d_drawCollision.getValue()) return;
    vparams->drawTool()->disableLighting();
    for (const auto& pair : d_output.getValue())
        vparams->drawTool()->drawLine(pair.first->getPosition(), pair.second->getPosition(), type::RGBAColor(0, 1, 0, 1));
}

}  // namespace sofa::collisionalgorithm
