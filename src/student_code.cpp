#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> newVector = std::vector<Vector2D>();

    for (int i = 0; i < points.size() - 1; i++) {
        float t = this->t;
        Vector2D newPoint = (1 - t) * points[i] + t * points[i + 1];
        newVector.push_back(newPoint);
    }

    return newVector;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> newVector = std::vector<Vector3D>();

      for (int i = 0; i < points.size() - 1; i++) {
          Vector3D newPoint = (1 - t) * points[i] + t * points[i + 1];
          newVector.push_back(newPoint);
      }

      return newVector;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> interPoints = points;
    while (interPoints.size() > 1) {
        interPoints = evaluateStep(interPoints, t);
    }
    return interPoints.front();
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector< std::vector<Vector3D> > controlPoints = this->controlPoints;
    std::vector<Vector3D> finalVector = std::vector<Vector3D>();

    for (int i = 0; i < controlPoints.size(); i++) {
        std::vector<Vector3D> oneVector = controlPoints[i];
        Vector3D newPoint = evaluate1D(oneVector, u);
        finalVector.push_back(newPoint);
    }

    return evaluate1D(finalVector, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    HalfedgeCIter h = this->halfedge();      // get the outgoing half-edge of the vertex
    Vector3D sum = Vector3D();
    int count = 0;
    do {
        HalfedgeCIter h_twin = h->twin(); // get the opposite half-edge
        VertexCIter v = h_twin->vertex(); // vertex is the 'source' of the half-edge, so
        FaceCIter f = h_twin->face();
        sum += f->normal();
        count += 1;
        h = h_twin->next();               // move to the next outgoing half-edge of the vertex
    } while (h != this->halfedge());
    return sum / count;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary()) {
        return e0;
    }

    // Getting all the halfedges in the pair of triangles

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    // Vertices

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    // Edges

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    // Faces

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    // Reassignment Part

    // Vertex
    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h1;
    v3->halfedge() = h0;

    // Edge
    e0->halfedge() = h0;
    e1->halfedge() = h5;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;

    // Face
    f0->halfedge() = h0;
    f1->halfedge() = h3;

    // Half-Edge
    h0->setNeighbors(h1, h3, v3, e0, f0);

    h1->setNeighbors(h2, h7, v2, e2, f0);

    h2->setNeighbors(h0, h8, v0, e3, f0);

    h3->setNeighbors(h4, h0, v2, e0, f1);

    h4->setNeighbors(h5, h9, v3, e4, f1);

    h5->setNeighbors(h3, h6, v1, e1, f1);

    h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());

    h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());

    h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());

    h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    if (e0->isBoundary()) {
        return VertexIter();
    }

    // Getting all the halfedges in the pair of triangles

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    // Vertices

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    // Edges

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    // Faces

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    // New elements
    HalfedgeIter hn1 = newHalfedge();
    HalfedgeIter hn2 = newHalfedge();
    HalfedgeIter hn3 = newHalfedge();
    HalfedgeIter hn4 = newHalfedge();
    HalfedgeIter hn5 = newHalfedge();
    HalfedgeIter hn6 = newHalfedge();

    FaceIter fn1 = newFace();
    FaceIter fn2 = newFace();

    VertexIter vn = newVertex();
    vn->position = (v0->position + v1->position) / 2;

    EdgeIter en1 = newEdge();
    EdgeIter en2 = newEdge();
    EdgeIter en3 = newEdge();
    EdgeIter en4 = newEdge();

    EdgeIter olde0 = e0;
    e0 = en4;

    // Reassignment Part

    // Vertex
    v0->halfedge() = hn4;
    v1->halfedge() = h3;
    v2->halfedge() = hn6;
    v3->halfedge() = hn2;
    vn->halfedge() = h0;

    // Edge
    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h2;
    e3->halfedge() = h4;
    e4->halfedge() = h5;
    en1->halfedge() = hn1;
    en2->halfedge() = hn3;
    en3->halfedge() = hn5;

    // Face
    f0->halfedge() = h0;
    f1->halfedge() = h3;
    fn1->halfedge() = hn3;
    fn2->halfedge() = hn4;

    // Halfedge setNeighbors(next, twin, vertex, edge, face)
    h0->setNeighbors(h1, h3, vn, e0, f0);

    h1->setNeighbors(hn6, h6, v1, e1, f0);

    h2->setNeighbors(hn4, h7, v2, e2, fn2);

    h3->setNeighbors(hn1, h0, v1, e0, f1);

    h4->setNeighbors(hn2, h8, v0, e3, fn1);

    h5->setNeighbors(h3, h9, v3, e4, f1);

    h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());

    h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());

    h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());

    h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());

    hn1->setNeighbors(h5, hn2, vn, en1, f1);

    hn2->setNeighbors(hn3, hn1, v3, en1, fn1);

    hn3->setNeighbors(h4, hn4, vn, en2, fn1);

    hn4->setNeighbors(hn5, hn3, v0, en2, fn2);

    hn5->setNeighbors(h2, hn6, vn, en3, fn2);

    hn6->setNeighbors(h0, hn5, v2, en3, f0);

    // Delete old e0
    deleteEdge(olde0);

    return vn;
  }
  
  VertexIter HalfedgeMesh::collapseEdge(EdgeIter e0)
  {
      if (e0->isBoundary()) {
          return VertexIter();
      }

      // Phase I

      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();

      HalfedgeIter h3 = h2->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      HalfedgeIter h6 = h5->twin();
      HalfedgeIter h7 = h6->next();
      HalfedgeIter h8 = h7->next();

      HalfedgeIter h9 = h8->twin();
      HalfedgeIter h10 = h9->next();
      HalfedgeIter h11 = h10->next();

      HalfedgeIter h12 = h11->twin();
      HalfedgeIter h13 = h12->next();
      HalfedgeIter h14 = h13->next();

      HalfedgeIter h15 = h14->twin();
      HalfedgeIter h16 = h15->next();
      HalfedgeIter h17 = h16->next();

      HalfedgeIter h18 = h16->twin();
      HalfedgeIter h19 = h18->next();
      HalfedgeIter h20 = h19->next();

      HalfedgeIter h21 = h20->twin();
      HalfedgeIter h22 = h21->next();
      HalfedgeIter h23 = h22->next();

      HalfedgeIter h24 = h23->twin();
      HalfedgeIter h25 = h24->next();
      HalfedgeIter h26 = h25->next();

      HalfedgeIter h27 = h26->twin();
      HalfedgeIter h28 = h27->next();
      HalfedgeIter h29 = h28->next();

      HalfedgeIter h30 = h28->twin();
      HalfedgeIter h31 = h25->twin();
      HalfedgeIter h32 = h22->twin();
      HalfedgeIter h33 = h19->twin();
      HalfedgeIter h34 = h13->twin();
      HalfedgeIter h35 = h10->twin();
      HalfedgeIter h36 = h7->twin();
      HalfedgeIter h37 = h4->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      VertexIter v4 = h8->vertex();
      VertexIter v5 = h11->vertex();
      VertexIter v6 = h14->vertex();
      VertexIter v7 = h20->vertex();
      VertexIter v8 = h23->vertex();
      VertexIter v9 = h26->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      EdgeIter e5 = h7->edge();
      EdgeIter e6 = h8->edge();
      EdgeIter e7 = h10->edge();
      EdgeIter e8 = h11->edge();
      EdgeIter e9 = h13->edge();
      EdgeIter e10 = h14->edge();
      EdgeIter e11 = h16->edge();
      EdgeIter e12 = h19->edge();
      EdgeIter e13 = h20->edge();
      EdgeIter e14 = h22->edge();
      EdgeIter e15 = h23->edge();
      EdgeIter e16 = h25->edge();
      EdgeIter e17 = h26->edge();
      EdgeIter e18 = h28->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      FaceIter f2 = h6->face();
      FaceIter f3 = h9->face();
      FaceIter f4 = h12->face();
      FaceIter f5 = h15->face();
      FaceIter f6 = h18->face();
      FaceIter f7 = h21->face();
      FaceIter f8 = h24->face();
      FaceIter f9 = h27->face();

      // phase II

      h3->setNeighbors(h4, h29, v0, e2, f1);
      h14->setNeighbors(h12, h18, v6, e10, f4);
      h18->setNeighbors(h19, h14, v0, e10, f6);
      h21->setNeighbors(h22, h20, v0, e13, f7);
      h24->setNeighbors(h25, h23, v0, e15, f8);
      h27->setNeighbors(h28, h26, v0, e17, f9);
      h29->setNeighbors(h27, h3, v2, e2, f9);

      v0->halfedge() = h3;
      v2->halfedge() = h29;
      v6->halfedge() = h14;

      e2->halfedge() = h3;
      e10->halfedge() = h14;

      deleteHalfedge(h0);
      deleteHalfedge(h1);
      deleteHalfedge(h2);
      deleteHalfedge(h15);
      deleteHalfedge(h16);
      deleteHalfedge(h17);

      deleteVertex(v1);

      deleteEdge(e0);
      deleteEdge(e1);
      deleteEdge(e11);

      deleteFace(f0);
      deleteFace(f5);

      return v0;
  }

  Vector3D neighbor_position_summer(VertexIter v) {
      Vector3D sum = Vector3D();
      HalfedgeCIter h = v->halfedge();      // get the outgoing half-edge of the vertex
      do {
          HalfedgeCIter h_twin = h->twin(); // get the opposite half-edge
          VertexCIter v = h_twin->vertex(); // vertex is the 'source' of the half-edge, so
          // h->vertex() is v, whereas h_twin->vertex()
          // is the neighboring vertex

          sum += v->position;

          h = h_twin->next();               // move to the next outgoing half-edge of the vertex
      } while (h != v->halfedge());          // keep going until we are back where we were

      return sum;
  }

  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        float n = v->degree();
        float u = (n == 3.0) ? (3.0 / 16.0) : 3.0 / (8.0 * n);
        Vector3D original_neighbor_position_sum = neighbor_position_summer(v);
        v->newPosition = (1.0 - n * u) * v->position + u * original_neighbor_position_sum;
        v->isNew = false;
    }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
        HalfedgeCIter h_edge = e->halfedge();
        HalfedgeCIter h_twin = h_edge->twin();
        VertexCIter A = h_edge->vertex();
        VertexCIter B = h_twin->vertex();
        HalfedgeCIter c_edge = h_edge->next()->twin();
        HalfedgeCIter d_edge = h_twin->next()->twin();
        VertexCIter C = c_edge->vertex();
        VertexCIter D = d_edge->vertex();

        e->newPosition = 3.0 / 8.0 * (A->position + B->position) + 1.0 / 8.0 * (C->position + D->position);
        e->isNew = false;
    }
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    // iterate over all edges in the mesh
    EdgeIter e = mesh.edgesBegin();
    int edgeMax = mesh.nEdges();
    int count = 0;
    while (e != mesh.edgesEnd()) {

        // get the next edge NOW!
        EdgeIter nextEdge = e;
        nextEdge++;

        // now, even if splitting the edge deletes it ...
        if (count < edgeMax && !e->isBoundary()) {
            // Get the next next halfedge of the edge
            HalfedgeIter prevHalfEdge = e->halfedge()->next()->next();
            HalfedgeIter prevTwin = e->halfedge()->twin()->next()->next();

            // Split
            VertexIter newV = mesh.splitEdge(e);
            newV->isNew = true;
            newV->newPosition = e->newPosition;

            // Get the new halfedge with prevHalfedge, this is the new halfedge that lies on the original edge
            // Do not flip it, so set it's isNew to false
            prevHalfEdge->next()->edge()->isNew = false;
            // Do the same for its twin
            prevTwin->next()->edge()->isNew = false;

            // Then, set the perpendicular edge of these two edges to true
            HalfedgeIter perpHalfEdge = prevHalfEdge->next()->next();
            HalfedgeIter perpTwin = prevTwin->next()->next();

            perpHalfEdge->edge()->isNew = true;
            perpTwin->edge()->isNew = true;
        }

        // ... we still have a valid reference to the next edge
        count++;
        e = nextEdge;
    }
    
    // 4. Flip any new edge that connects an old and new vertex.

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
        if (e->isNew) {
            HalfedgeCIter h_edge = e->halfedge();
            HalfedgeCIter h_twin = h_edge->twin();
            VertexCIter A = h_edge->vertex();
            VertexCIter B = h_twin->vertex();
            if ((A->isNew && !B->isNew) || (!A->isNew && B->isNew)) {
                mesh.flipEdge(e);
            }
        }
    }

    // 5. Copy the new vertex positions into final Vertex::position.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        v->position = v->newPosition;
    }

  }
}