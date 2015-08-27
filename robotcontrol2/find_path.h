#ifndef _find_path_h
#define _find_path_h

#include <boost/operators.hpp>
#include <cstdint>

struct size;

struct point :
    boost::equality_comparable<point,
    boost::additive< point, size > >
{
	int x;
	int y;
    
    point(int _x, int _y) : x(_x), y(_y) {}

	friend point operator+=(point& pt, size const& sz);
	friend point operator-=(point& pt, size const& sz);
    
    friend size operator-(point& lhs, point const& rhs);
    
    friend bool operator==(point const& lhs, point const& rhs);
};

struct size {
	int x;
	int y;
    
    int Abs() const;
};

struct SMap {
    std::uint8_t const* m_pbImage;
    std::size_t m_cbBytesPerRow;
	size m_sz;

	bool contains(point const& pt) const;
	std::uint8_t operator[](point const& pt) const;
	bool is_free(point const& pt) const;
};

enum dir {
	begin = 0,
	left = 0,
	up = 1,
	right = 2,
	down = 3,
	end = 4
};
#endif