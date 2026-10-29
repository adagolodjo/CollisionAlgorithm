#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <CollisionAlgorithm/DataDetectionOutput.h>
#include <CollisionAlgorithm/config.h>

namespace sofa::collisionalgorithm {

/*!
 * \brief For samples taken on the elements of a geometry (`from`), finds the closest proximity on another geometry
 * (`dest`) and outputs the pairs (sample, closest) that are within `distance`.
 *
 * It is the contact detection of a thin object (rod, wire, needle shaft) against a surface, without the puncture and
 * insertion phases of InsertionAlgorithm.  Per element of `from`:
 *  - `samples` points are taken at the middle of `samples` equal parts of the element (edges only; any other kind of
 *    element, a point for instance, gives one sample, itself or its centre);
 *  - the closest proximity on `dest` is searched for each sample, within `distance` and accepted by the filters;
 *  - with `projective` (edges only), the samples are replaced by one pair per edge, placed where the edge is closest
 *    to the surface: the distance to `dest` is evaluated at both ends and in the middle, then refined by
 *    `projectiveIterations` golden-section steps in the best bracket (a few closest-proximity queries per edge that is
 *    within reach, a single one for the others).
 * Use a PointGeometry on the same positions as `from` to also test the nodes of an EdgeGeometry.
 */
class SOFA_COLLISIONALGORITHM_API ClosestProximityAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(ClosestProximityAlgorithm, BaseAlgorithm);

    typedef core::objectmodel::SingleLink<ClosestProximityAlgorithm, BaseGeometry,
                                          BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK>
        GeomLink;
    typedef DetectionOutput<BaseProximity, BaseProximity> AlgorithmOutput;

    GeomLink l_from, l_dest;
    Data<AlgorithmOutput> d_output;
    Data<int> d_samples;
    Data<bool> d_projective;
    Data<int> d_projectiveIterations;
    Data<SReal> d_distance;
    Data<bool> d_drawCollision;

    ClosestProximityAlgorithm();

    void doDetection() override;

    void draw(const core::visual::VisualParams* vparams) override;

    /// Smallest of `distance` and of the `distance` of the DistanceFilters of this algorithm.
    double getMaxDistance() const;
};

}  // namespace sofa::collisionalgorithm
