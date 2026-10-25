#pragma once

#include <CollisionAlgorithm/BaseAABBBroadPhase.h>
#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseElement.h>
#include <sofa/helper/AdvancedTimer.h>

namespace sofa::collisionalgorithm {

class SOFA_COLLISIONALGORITHM_API FullAABBBroadPhase : public BaseAABBBroadPhase {
public:
    SOFA_CLASS(FullAABBBroadPhase, BaseAABBBroadPhase);

    FullAABBBroadPhase() {
    }

    void updateData() override {
    }

    const std::set<BaseElement::SPtr>& getElementSet(unsigned i, unsigned j, unsigned k) const override {
        static std::set<BaseElement::SPtr> empty;
        if (i >= static_cast<unsigned>(m_nbox[0]) || j >= static_cast<unsigned>(m_nbox[1]) ||
            k >= static_cast<unsigned>(m_nbox[2]))
            return empty;

        return m_indexedElement[i][j][k];
    }

    void newContainer() override {
        m_indexedElement.clear();

        m_indexedElement.resize(m_nbox[0]);
        for (int i = 0; i < m_nbox[0]; i++) {
            m_indexedElement[i].resize(m_nbox[1]);
            for (int j = 0; j < m_nbox[1]; j++) {
                m_indexedElement[i][j].resize(m_nbox[2]);
            }
        }
        //        std::cout << "test size vector : " << m_indexedElement.size() << " " <<  m_indexedElement[0].size() <<
        //        " " << m_indexedElement[0][0].size() << " " << std::endl;
    }

    void addElement(int i, int j, int k, BaseElement::SPtr elmt) override {
        if (i < 0 || j < 0 || k < 0 || i >= m_nbox[0] || j >= m_nbox[1] || k >= m_nbox[2]) return;

        m_indexedElement[i][j][k].insert(elmt);
    }

protected:
    std::vector<std::vector<std::vector<std::set<BaseElement::SPtr>>>> m_indexedElement;
    type::Vec<2, size_t> m_offset;
};

}  // namespace sofa::collisionalgorithm
