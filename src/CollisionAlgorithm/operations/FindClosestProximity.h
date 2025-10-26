#pragma once

#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <limits.h>
#include <CollisionAlgorithm/operations/Project.h>

namespace sofa::collisionalgorithm::Operations::FindClosestProximity {

typedef BaseProximity::SPtr Result;
typedef std::function<bool(const BaseProximity::SPtr&,const BaseProximity::SPtr&)> FilterFUNC;

//Specific operation to find the closest point on a geometry (the code is in the c++ class)
class SOFA_COLLISIONALGORITHM_API Operation : public Operations::GenericOperation<Operation,//operation type
                                                      Result, // default return
                                                      const BaseProximity::SPtr &,BaseGeometry *, Operations::Project::FUNC, FilterFUNC // parameters
                                                      > {
public:

    static ElementIterator::SPtr broadPhaseIterator(BaseProximity::SPtr prox, BaseGeometry::BroadPhase * broadphase) {
        //old params : type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d
        type::Vec3i nbox = broadphase->getNbox();

        //take the first broad phase...
        type::Vec3 P = prox->getPosition();

        type::Vec3i cbox = broadphase->getBoxCoord(P);

        //project the box in the bounding box of the object
        //search with the closest box in bbox
        cbox[0] = std::max(0,std::min(nbox[0],cbox[0]));
        cbox[1] = std::max(0,std::min(nbox[1],cbox[1]));
        cbox[2] = std::max(0,std::min(nbox[2],cbox[2]));

    //        int max = 0;
    //        for (int i = 0 ; i < 3 ; i++) {
    //            max = std::max (max, cbox[i]) ;
    //            max = std::max (max, nbox[i]-cbox[i]) ;
    //        }

        std::set<BaseElement::SPtr> selectedElements;
        
        // Unified loop that iterates over expanding shells around the center box
        // This replaces 6 separate loop blocks with a single, clearer implementation
        int maxDistance = std::max({nbox[0] - cbox[0], cbox[0], 
                                    nbox[1] - cbox[1], cbox[1],
                                    nbox[2] - cbox[2], cbox[2]}) + 1;

        for (int d = 0; d <= maxDistance && selectedElements.empty(); d++) {
            // Iterate over the shell at distance d from the center box
            for (int i = -d; i <= d; i++) {
                for (int j = -d; j <= d; j++) {
                    for (int k = -d; k <= d; k++) {
                        // Only process cells on the boundary of the shell (not interior)
                        // For d=0, process only the center cell
                        if (d > 0 && std::abs(i) != d && std::abs(j) != d && std::abs(k) != d)
                            continue;
                        
                        int bi = cbox[0] + i;
                        int bj = cbox[1] + j;
                        int bk = cbox[2] + k;
                        
                        // Check bounds
                        if (bi < 0 || bi > nbox[0] || 
                            bj < 0 || bj > nbox[1] || 
                            bk < 0 || bk > nbox[2])
                            continue;
                        
                        const std::set<BaseElement::SPtr>& elmts = 
                            broadphase->getElementSet(bi, bj, bk);
                        selectedElements.insert(elmts.cbegin(), elmts.cend());
                    }
                }
            }
        }

        return ElementIterator::SPtr(new TDefaultElementIterator_copy(selectedElements));
    }

    BaseProximity::SPtr defaultFunc(const BaseProximity::SPtr & prox, BaseGeometry * geometry, Operations::Project::FUNC projectOp,BaseAlgorithm::FilterFUNC filter) const override {
        const ElementIterator::SPtr & itdest = (geometry->getBroadPhase()) ?
                                               broadPhaseIterator(prox, geometry->getBroadPhase()) :
                                               geometry->begin();

        return doFindClosesPoint(prox, itdest, projectOp, filter);
    }

private:
    BaseProximity::SPtr doFindClosesPoint(const BaseProximity::SPtr & prox, ElementIterator::SPtr itdest, Operations::Project::FUNC projectOp,BaseAlgorithm::FilterFUNC filter) const {
        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr res = NULL;

//        type::Vec3 P = prox->getPosition();

        for (; ! itdest->end();itdest++) {
            auto edest = itdest->element();
            if (edest == nullptr) continue;

            Operations::Project::Result result = projectOp(prox->getPosition(),edest);

            if (result.prox == NULL) continue;

            if (result.distance< min_dist) {
				auto normalizedProx = result.prox->copy();
				normalizedProx->normalize();

				if (! filter(prox,normalizedProx)) continue;


				res = result.prox;
                min_dist = result.distance;
            }
        }

        return res;
    }
};

typedef Operation::FUNC FUNC;

}

