#pragma once

#include <CollisionAlgorithm/BaseAABBBroadPhase.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <limits.h>

#include <algorithm>
#include <unordered_set>

namespace sofa::collisionalgorithm::Operations::FindClosestProximity {

typedef BaseProximity::SPtr Result;
typedef std::function<bool(const BaseProximity::SPtr &, const BaseProximity::SPtr &)> FilterFUNC;

// Specific operation to find the closest point on a geometry (the code is in the c++ class)
class SOFA_COLLISIONALGORITHM_API Operation
    : public Operations::GenericOperation<Operation,  // operation type
                                          Result,     // default return
                                          const BaseProximity::SPtr &, BaseGeometry *, Operations::Project::FUNC,
                                          FilterFUNC, double  // parameters (the last one: farthest distance a result may have)
                                          > {
public:
    static ElementIterator::SPtr broadPhaseIterator(BaseProximity::SPtr prox, BaseGeometry::BroadPhase *broadphase) {
        // old params : type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d
        type::Vec3i nbox = broadphase->getNbox();
        if (nbox[0] <= 0 || nbox[1] <= 0 || nbox[2] <= 0) return ElementIterator::empty();

        // take the first broad phase...
        type::Vec3 P = prox->getPosition();

        type::Vec3i cbox = broadphase->getBoxCoord(P);

        // project the box in the bounding box of the object
        // search with the closest box in bbox
        cbox[0] = std::max(0, std::min(nbox[0] - 1, cbox[0]));
        cbox[1] = std::max(0, std::min(nbox[1] - 1, cbox[1]));
        cbox[2] = std::max(0, std::min(nbox[2] - 1, cbox[2]));

        //        int max = 0;
        //        for (int i = 0 ; i < 3 ; i++) {
        //            max = std::max (max, cbox[i]) ;
        //            max = std::max (max, nbox[i]-cbox[i]) ;
        //        }

        std::set<BaseElement::SPtr> selectedElements;

        // Unified loop that iterates over expanding shells around the center box
        // This replaces 6 separate loop blocks with a single, clearer implementation
        int maxDistance =
            std::max({nbox[0] - 1 - cbox[0], cbox[0], nbox[1] - 1 - cbox[1], cbox[1], nbox[2] - 1 - cbox[2], cbox[2]}) +
            1;

        for (int d = 0; d <= maxDistance && selectedElements.empty(); d++) {
            // Iterate over the shell at distance d from the center box
            for (int i = -d; i <= d; i++) {
                for (int j = -d; j <= d; j++) {
                    for (int k = -d; k <= d; k++) {
                        // Only process cells on the boundary of the shell (not interior)
                        // For d=0, process only the center cell
                        if (d > 0 && std::abs(i) != d && std::abs(j) != d && std::abs(k) != d) continue;

                        int bi = cbox[0] + i;
                        int bj = cbox[1] + j;
                        int bk = cbox[2] + k;

                        // Check bounds
                        if (bi < 0 || bi >= nbox[0] || bj < 0 || bj >= nbox[1] || bk < 0 || bk >= nbox[2]) continue;

                        const std::set<BaseElement::SPtr> &elmts = broadphase->getElementSet(bi, bj, bk);
                        selectedElements.insert(elmts.cbegin(), elmts.cend());
                    }
                }
            }
        }

        return ElementIterator::SPtr(new TDefaultElementIterator_copy(selectedElements));
    }

    BaseProximity::SPtr defaultFunc(const BaseProximity::SPtr &prox, BaseGeometry *geometry,
                                    Operations::Project::FUNC projectOp, BaseAlgorithm::FilterFUNC filter,
                                    double maxDistance) const override {
        BaseGeometry::BroadPhase *broadphase = geometry->getBroadPhase();

        // Regular grid: exact search, expanding the cells around the query until no farther cell can hold a closer element
        if (auto *grid = dynamic_cast<BaseAABBBroadPhase *>(broadphase)) {
            const type::Vec3 &cell = grid->getCellSize();
            double minCell = std::numeric_limits<double>::max();
            for (int i = 0; i < 3; i++)
                if (cell[i] > 0.0) minCell = std::min(minCell, cell[i]);
            if (minCell < std::numeric_limits<double>::max()) return doFindClosestInGrid(prox, grid, minCell, projectOp, filter, maxDistance);
        }

        const ElementIterator::SPtr &itdest = (broadphase) ? broadPhaseIterator(prox, broadphase) : geometry->begin();

        return doFindClosesPoint(prox, itdest, projectOp, filter);  // maxDistance is only used to prune the grid search
    }

private:
    /*!
     * Closest proximity found by visiting the grid cells in shells of increasing Chebyshev distance d around the cell of
     * the query. Every element not seen in the shells 0..d-1 lies in cells at least d cells away, hence at least
     * (d-1) * cell size away from the query, and sqrt(((d-1) * cell size)^2 + e^2) when the query is outside the grid, e being its distance to the grid box. The search stops
     * as soon as that bound exceeds the distance of the best accepted proximity or, while none was accepted, of the nearest
     * candidate (accepted or not): a filter that rejects the nearest candidate therefore does not trigger a search of the whole grid.
     * The previous implementation stopped at the first non-empty shell and could return a farther element than one in the
     * next cell.
     */
    BaseProximity::SPtr doFindClosestInGrid(const BaseProximity::SPtr &prox, BaseAABBBroadPhase *grid, double minCell,
                                            Operations::Project::FUNC projectOp, BaseAlgorithm::FilterFUNC filter,
                                            double maxDistance) const {
        const type::Vec3i nbox = grid->getNbox();
        if (nbox[0] <= 0 || nbox[1] <= 0 || nbox[2] <= 0) return nullptr;

        const type::Vec3 P = prox->getPosition();
        type::Vec3i cbox = grid->getBoxCoord(P);
        for (int i = 0; i < 3; i++) cbox[i] = std::max(0, std::min(nbox[i] - 1, cbox[i]));

        // distance from the query to the grid box (0 if inside): the elements all lie in the box
        const type::Vec3 &bmin = grid->getMin();
        const type::Vec3 &cellSize = grid->getCellSize();
        double outside2 = 0.0;
        for (int i = 0; i < 3; i++) {
            const double lo = bmin[i], hi = bmin[i] + nbox[i] * cellSize[i];
            const double e = (P[i] < lo) ? lo - P[i] : (P[i] > hi ? P[i] - hi : 0.0);
            outside2 += e * e;
        }

        const int maxShell =
            std::max({nbox[0] - 1 - cbox[0], cbox[0], nbox[1] - 1 - cbox[1], cbox[1], nbox[2] - 1 - cbox[2], cbox[2]});

        std::unordered_set<const BaseElement *> seen;
        BaseProximity::SPtr res = nullptr;
        double bestDistance = std::numeric_limits<double>::max();
        double nearestCandidate = std::numeric_limits<double>::max();

        // Distance beyond which nothing can be accepted or improve the result: the caller's maximum distance, the best
        // accepted distance or, while nothing was accepted and no maximum is known, the nearest candidate.
        auto bound = [&]() {
            return std::min(maxDistance, (res != nullptr) ? bestDistance : nearestCandidate);
        };

        for (int d = 0; d <= maxShell; d++) {
            const double reach = (d - 1) * minCell;  // lower bound of the distance to any element not seen yet, ...
            if (reach > 0.0 && reach * reach + outside2 >= bound() * bound()) break;  // ... larger when the query is outside the grid

            for (int i = -d; i <= d; i++) {
                for (int j = -d; j <= d; j++) {
                    for (int k = -d; k <= d; k++) {
                        if (d > 0 && std::abs(i) != d && std::abs(j) != d && std::abs(k) != d) continue;

                        const int bi = cbox[0] + i, bj = cbox[1] + j, bk = cbox[2] + k;
                        if (bi < 0 || bi >= nbox[0] || bj < 0 || bj >= nbox[1] || bk < 0 || bk >= nbox[2]) continue;

                        // skip the cells that cannot hold anything closer than the bound
                        double cell2 = 0.0;
                        const int index[3] = {bi, bj, bk};
                        for (int a = 0; a < 3; a++) {
                            const double lo = bmin[a] + index[a] * cellSize[a], hi = lo + cellSize[a];
                            const double e = (P[a] < lo) ? lo - P[a] : (P[a] > hi ? P[a] - hi : 0.0);
                            cell2 += e * e;
                        }
                        if (cell2 >= bound() * bound()) continue;

                        for (const BaseElement::SPtr &edest : grid->getElementSet(bi, bj, bk)) {
                            if (edest == nullptr || !seen.insert(edest.get()).second) continue;

                            const Operations::Project::Result result = projectOp(P, edest);
                            if (result.prox == nullptr) continue;

                            nearestCandidate = std::min(nearestCandidate, result.distance);
                            if (result.distance >= bestDistance || result.distance > maxDistance) continue;

                            auto normalizedProx = result.prox->copy();
                            normalizedProx->normalize();
                            if (!filter(prox, normalizedProx)) continue;

                            res = result.prox;
                            bestDistance = result.distance;
                        }
                    }
                }
            }
        }
        return res;
    }

    BaseProximity::SPtr doFindClosesPoint(const BaseProximity::SPtr &prox, ElementIterator::SPtr itdest,
                                          Operations::Project::FUNC projectOp, BaseAlgorithm::FilterFUNC filter) const {
        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr res = NULL;

        //        type::Vec3 P = prox->getPosition();

        for (; !itdest->end(); itdest++) {
            auto edest = itdest->element();
            if (edest == nullptr) continue;

            Operations::Project::Result result = projectOp(prox->getPosition(), edest);

            if (result.prox == NULL) continue;

            if (result.distance < min_dist) {
                auto normalizedProx = result.prox->copy();
                normalizedProx->normalize();

                if (!filter(prox, normalizedProx)) continue;

                res = result.prox;
                min_dist = result.distance;
            }
        }

        return res;
    }
};

typedef Operation::FUNC FUNC;

}  // namespace sofa::collisionalgorithm::Operations::FindClosestProximity
