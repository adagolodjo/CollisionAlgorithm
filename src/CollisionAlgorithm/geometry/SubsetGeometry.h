#pragma once

#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/proximity/PointProximity.h>
#include <CollisionAlgorithm/toolbox/PointToolBox.h>

#include <algorithm>

namespace sofa::collisionalgorithm {

template<class DataTypes>
class SubsetGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PointElement ELEMENT;
    typedef SubsetGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const PointElement * elmt)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;
    Data<type::vector<int> > d_indices;

    SubsetGeometry()
      : l_geometry(initLink("wholeGeometry", "link to topology"))
      , d_indices(initData(&d_indices,"indices", "Indices to keep"))
    {
      l_geometry.setPath("@.");
    }
//    type::Vec3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

	void init() override
	{
		type::vector<int> indices = d_indices.getValue();

		std::sort(indices.begin(), indices.end());
		indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

		m_elements.clear();

		if (l_geometry == nullptr)
		{
			msg_error() << "SubsetGeometry has no linked geometry, no element will be used.";
			return;
		}

		auto iter = l_geometry->begin();
		int id = 0;
		for (int keepId : indices)
		{
			if (keepId < 0 || keepId < id)
			{
				msg_warning() << "Invalid or duplicated index " << keepId << ", skipping.";
				continue;
			}

			while (id < keepId && !iter->end())
			{
				++id;
				iter++;
			}

			if (iter->end())
			{
				msg_warning() << "Index " << keepId
				              << " is out of range, ignoring the remaining indices.";
				break;
			}

			m_elements.push_back(iter->element());
		}
	}


    void prepareDetection() override {}

    ElementIterator::SPtr begin(unsigned id = 0) const override {
        return ElementIterator::SPtr(new TDefaultElementIterator_ref(m_elements,id));
    }

private:
    std::vector<BaseElement::SPtr> m_elements;
};

}
