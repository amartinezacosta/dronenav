#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include <vector>

class Vertex
{
    public:
    Vertex(double x, double y, double z, int depth) :
        m_x(x), m_y(y), m_z(z), m_depth(depth)
    {
    }

    double get_x(void) const { return m_x;}
    double get_y(void) const { return m_y;}
    double get_z(void) const { return m_z;}
    int get_depth(void) const { return m_depth; }
    void add_neighbor(Vertex *neighbor){ m_neighbors.push_back(neighbor); }

    std::vector<Vertex*> get_neighbors(void){ return m_neighbors; }

    bool operator==(const Vertex& v) const
    {
        return ( (v.get_x() == m_x) &&
                 (v.get_y()  == m_y) &&
                 (v.get_z() == m_z) &&
                 (v.get_depth() == m_depth));
    }

    private:
    double m_x, m_y, m_z;
    int m_depth;
    std::vector<Vertex*> m_neighbors;
};

class AStarVertex : public Vertex
{
    public:
    AStarVertex(double x, double y, double z, int depth) :
        Vertex(x, y, z, depth),
        m_f(INFINITY),
        m_g(INFINITY),
        m_h(INFINITY),
        m_parent(nullptr)
    {
    }

    void set_f(double f){ m_f = f;}
    void set_g(double g){ m_g = g;}
    void set_h(double h){ m_h = h;}
    void set_parent(AStarVertex *parent){ m_parent = parent;}

    double get_f(void) const { return m_f; }
    double get_g(void) const { return m_g; }
    double get_h(void) const { return m_h; }
    AStarVertex* get_parent(void) const { return m_parent; }

    private:
    double m_f, m_g, m_h;
    AStarVertex *m_parent;
};

#endif