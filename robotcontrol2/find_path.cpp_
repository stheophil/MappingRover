#include "find_path.h"

#include <assert.h>
#include <vector>
#include <tuple>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/ordered_index.hpp>

#include <boost/range.hpp>
#include <boost/range/algorithm.hpp>

#define VERIFY(expr) (assert(expr))
#define ASSERT(expr) (assert(expr))


// size

int size::Abs() const {
    return static_cast<int>(sqrt(x*x + y*y));
}

// Map
bool SMap::contains(point const& pt) const {
	return 0<=pt.x && 0<=pt.y
		&& pt.x < m_sz.x && pt.y < m_sz.y;
}

std::uint8_t SMap::operator[](point const& pt) const {
	return m_pbImage[m_cbBytesPerRow * pt.y + pt.x];
}

bool SMap::is_free(point const& pt) const {
    // Performance is sensitive to number of pixel considered free.
    return contains(pt) && std::numeric_limits<std::uint8_t>::max()/3 < (*this)[pt];
}

// A* path finding algorithm (with penalty for turning instead of driving straight)

static const int c_nTurnPenalty = 10;
static constexpr size c_aszTransform[] = {
    size{-1, 0}, size{0, -1}, size{1, 0}, size{0, 1}
};

struct SPosition {
	point m_pt;
	dir m_dir;
    
	int m_nWayCost;
    int m_nCost;
    
    SPosition(point const& pt, dir const& d, int nWayCost, point const& ptGoal)
        : m_pt(pt),
        m_dir(d),
        m_nWayCost(nWayCost),
        m_nCost(nWayCost + std::abs(ptGoal.x - pt.x) + std::abs(ptGoal.y - pt.y)) // Manhattan distance heuristic
    {}

	typedef std::tuple<int, int, int> key_type;
	key_type key() const {
		return std::make_tuple(m_pt.x, m_pt.y, static_cast<int>(m_dir));
	}

    point predecessor() const {
        return m_pt - c_aszTransform[m_dir];
    }
    
	template<typename Func>
	void for_each_neighbor(point const& ptGoal, Func fn) {
        for(dir d = dir::begin; d<dir::end; d = static_cast<dir>(static_cast<int>(d)+1)) {
			fn( SPosition{ 
				m_pt + c_aszTransform[d],
				d,
				m_nWayCost + (m_dir==d ? 1 : c_nTurnPenalty),
                ptGoal
			} );
		}
	}
};

struct SCompareKeyType : std::less<SPosition::key_type> {
    using std::less<SPosition::key_type>::operator();

    int compare(SPosition::key_type const& lhs, point const& rhs) const {
        if(std::get<0>(lhs) < rhs.x) return -1;
        if(std::get<0>(lhs) > rhs.x) return 1;
        
        if(std::get<1>(lhs) < rhs.y) return -1;
        if(std::get<1>(lhs) > rhs.y) return 1;
        
        return 0;
    }
    
    bool operator()(SPosition::key_type const& lhs, point const& rhs) const {
        return compare(lhs, rhs)<0;
    }
    
    bool operator()(point const& lhs, SPosition::key_type const& rhs) const {
        return 0<compare(rhs, lhs);
    }
};

// Output is in reverse order, from goal to start
std::vector<SPosition> find_path(point const& ptStart, dir d, point const& ptGoal, SMap const& map) {
    // TODO: Consider time-out criterion, or track point with closest heuristic distance to ptGoal
    // and bail out if closest open point is much further than closest point
    // Give max path length?
	boost::multi_index_container< 
		SPosition, 
        boost::multi_index::indexed_by<
			boost::multi_index::ordered_non_unique<boost::multi_index::member<SPosition,int,&SPosition::m_nCost>>,
			boost::multi_index::hashed_unique<boost::multi_index::const_mem_fun<SPosition,SPosition::key_type,&SPosition::key>>
		> 
	> setposOpen;
	boost::multi_index_container< 
		SPosition, 
		boost::multi_index::indexed_by<
			boost::multi_index::ordered_unique<boost::multi_index::const_mem_fun<SPosition,SPosition::key_type,&SPosition::key>, SCompareKeyType>
		> 
	> setposVisited;

	setposOpen.insert( SPosition{ptStart, d, 0, ptGoal} );

	while(!setposOpen.empty()) {
		// Remove position with lowest cost from open-list
		SPosition pos = *setposOpen.begin();
		setposOpen.erase(setposOpen.begin());

        if(pos.m_pt == ptGoal) {
            // Go back least-cost route to ptStart
            std::vector<SPosition> vecposPath = { pos };
            while(ptStart != vecposPath.back().m_pt) {
                auto rngposPred = setposVisited.equal_range(vecposPath.back().predecessor());
                auto itposMinimum = boost::min_element(rngposPred, [](SPosition const& lhs, SPosition const& rhs) {
                    return lhs.m_nCost < rhs.m_nCost;
                });
                ASSERT(itposMinimum!=setposVisited.end());
                if(1<vecposPath.size()
                && ((vecposPath.back().m_pt.x==(vecposPath.end()-2)->m_pt.x && vecposPath.back().m_pt.x==itposMinimum->m_pt.x)
                 || (vecposPath.back().m_pt.y==(vecposPath.end()-2)->m_pt.y && vecposPath.back().m_pt.y==itposMinimum->m_pt.y)))
                {
                    vecposPath.back() = *itposMinimum;
                } else {
                    vecposPath.emplace_back(*itposMinimum);
                }
            }
            return vecposPath;
        }
        VERIFY(setposVisited.insert(pos).second);
        
        pos.for_each_neighbor(ptGoal, [&](SPosition const& posNew) {
            if(map.is_free(posNew.m_pt)
            && setposVisited.find(posNew.key())==setposVisited.end())
            {
                // posNew is an unoccupied position on the map
                // that has not been visited yet
                auto itposOpen = setposOpen.get<1>().find(posNew.key());
                if(itposOpen==setposOpen.get<1>().end()) {
                    setposOpen.insert(posNew);
                } else {
                    VERIFY(setposOpen.get<1>().modify(itposOpen, [&](SPosition& posModify) {
                        posModify.m_nCost = std::min(posModify.m_nCost, posNew.m_nCost);
                    }));
                }
            }
        });
	}
    return std::vector<SPosition>();
}

extern "C" {
    // Objective-C / Swift interface
    int find_path(int nStartX, int nStartY,
                   int nDirection,
                   int nGoalX, int nGoalY,
                   unsigned char const* pbImage,
                   unsigned int cbBytesPerRow,
                   int nExtent,
                   void (^foreachPoint)(int nX, int nY))
    {
        ASSERT(dir::begin<=nDirection && nDirection<dir::end);
        auto vecposPath = find_path(point{nStartX, nStartY},
                                    static_cast<dir>(nDirection),
                                    point{nGoalX, nGoalY},
                                    SMap{pbImage, cbBytesPerRow, size{nExtent, nExtent}});
        boost::for_each(vecposPath,
                        [&](SPosition const& pos) {
                            foreachPoint(pos.m_pt.x, pos.m_pt.y);
                        });
        return vecposPath.empty() ? std::numeric_limits<int>::max() : vecposPath.front().m_nCost;
    }
}