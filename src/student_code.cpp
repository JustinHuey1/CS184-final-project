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
      // Phase I

      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();

      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      HalfedgeIter h1t = h1->twin();
      HalfedgeIter h2t = h2->twin();
      HalfedgeIter h4t = h4->twin();
      HalfedgeIter h5t = h5->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      // Detect if boundary or not manifold
      if (v0->isBoundary() || v1->isBoundary() || v0->degree() < 2 || v1->degree() < 2) {
          return this->verticesEnd();
      }

      // From https://stackoverflow.com/questions/27049163/mesh-simplification-edge-collapse-conditions
      // The collapsing edge has two incident vertices. For both vertices, gather the adjacent vertices and calculate 
      // the intersection of both sets. This intersection should contain exactly two vertices.
      set<VertexIter> v0Vertices;
      int count = 0;

      HalfedgeIter v0Edge = v0->halfedge();
      do {
          v0Vertices.insert(v0Edge->twin()->vertex());
          v0Edge = v0Edge->twin()->next();
      } while (v0Edge != v0->halfedge());

      // Loop through second
      HalfedgeIter v1Edge = v1->halfedge();
      do {
          if (v0Vertices.find(v1Edge->twin()->vertex()) != v0Vertices.end()) {
              count++;
          }
          v1Edge = v1Edge->twin()->next();
      } while (v1Edge != v1->halfedge());
      if (count != 2) {
          return this->verticesEnd();
      }

      // phase II
      // Editing part
      // Set all outgoing halfedges from v1 to v0
      HalfedgeIter h = v1->halfedge();      // get the outgoing half-edge of the vertex
      do {
          h->vertex() = v0; // Set h's vertex to v0
          h = h->twin()->next();               // move to the next outgoing half-edge of the vertex
      } while (h != v1->halfedge());

      v0->halfedge() = h2t;
      v2->halfedge() = h1t;
      v3->halfedge() = h4t;

      e2->halfedge() = h1t;
      e3->halfedge() = h4t;

      h1t->setNeighbors(h1t->next(), h2t, v2, e2, h1t->face());
      h2t->setNeighbors(h2t->next(), h1t, v0, e2, h2t->face());
      h4t->setNeighbors(h4t->next(), h5t, v3, e3, h4t->face());
      h5t->setNeighbors(h5t->next(), h4t, v0, e3, h5t->face());

      deleteHalfedge(h0);
      deleteHalfedge(h1);
      deleteHalfedge(h2);
      deleteHalfedge(h3);
      deleteHalfedge(h4);
      deleteHalfedge(h5);

      deleteVertex(v1);

      deleteEdge(e0);
      deleteEdge(e1);
      deleteEdge(e4);

      deleteFace(f0);
      deleteFace(f1);

      return v0;
  }

    void MeshResampler::simplification(HalfedgeMesh& mesh) {
        int target = 0; // Target # of triangles/faces
        MutablePriorityQueue<EdgeRecord> queue;
        
        for (FaceIter f = mesh.facesBegin(); f != mesh.facesEnd(); f++) {
            target += 1;
            Vector3D normal = f -> normal();
            Vector3D queryPosition = f -> halfedge() -> vertex() -> position;
            
            double a = normal.x;
            double b = normal.y;
            double c = normal.z;
            double d = -1.0 * dot(queryPosition, normal);
            
            double data[16] = {a * a, a * b, a * c, a * d, a * b, b * b, b * c, b * d,
                a * c, b * c, c * c, c * d, a * d, b * d, c * d, d * d};
            
            f -> quadric = Matrix4x4(data);
        }

        target /= 2.0;
        
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            Matrix4x4 sumQ = Matrix4x4();
            
            HalfedgeIter h = v->halfedge();      // get the outgoing half-edge of the vertex
            do {
                sumQ += h -> face()->quadric;

                h = h -> twin() ->next();               // move to the next outgoing half-edge of the vertex
            } while (h != v->halfedge());
                       
            v -> quadric = sumQ;
        }
        
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            EdgeRecord record = EdgeRecord(e);
            e->record = record;
            
            queue.insert(record);
        }
        // 3696 - 3668

        while (mesh.nFaces() > target) {
            if (queue.empty()) {
                break;
            }
            EdgeRecord record = queue.top();
            queue.pop();

            EdgeIter edge = record.edge;
            VertexIter onePoint = edge->halfedge()->vertex();
            VertexIter otherPoint = edge->halfedge()->twin()->vertex();
            Matrix4x4 newQuadric = onePoint->quadric + otherPoint->quadric;

            // Delete all records that neighbors edge
            HalfedgeIter h1 = onePoint->halfedge();
            do {
                EdgeIter h_edge = h1->edge();
                EdgeRecord e_record = h_edge->record;
                queue.remove(e_record);

                h1 = h1->twin()->next();
            } while (h1 != onePoint->halfedge());

            HalfedgeIter h2 = otherPoint->halfedge();
            do {
                EdgeIter h_edge = h2->edge();
                EdgeRecord e_record = h_edge->record;
                queue.remove(e_record);

                h2 = h2->twin()->next();
            } while (h2 != otherPoint->halfedge());

            // Then collapse
            VertexIter m = mesh.collapseEdge(edge);
            if (m == mesh.verticesEnd()) {
                continue;
            }
            m->quadric = newQuadric;
            m->position = record.optimalPoint;

            // Add any edges touching the new vertex into the queue
            HalfedgeIter h = m->halfedge();
            do {
                EdgeIter h_edge = h->edge();
                EdgeRecord new_record = EdgeRecord(h_edge);
                h_edge->record = new_record;
                queue.insert(new_record);

                h = h->twin()->next();
            } while (h != m->halfedge());
        }
    }

    EdgeRecord::EdgeRecord(EdgeIter& e) {
        VertexIter onePoint = e->halfedge()->vertex();
        VertexIter otherPoint = e->halfedge()->twin()->vertex();

        e->quadric = onePoint->quadric + otherPoint->quadric;

        double data[16] = {};

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                data[i * 4 + j] = e->quadric(i, j);
            }
        }
        
        data[12] = 0;
        data[13] = 0;
        data[14] = 0;
        data[15] = 1;

        Matrix4x4 A = Matrix4x4(data);
        Vector4D B = Vector4D(0.0, 0.0, 0.0, 1.0);

        // optimal point
        Vector4D x = A.inv() * B;

        double cost = dot(x, e->quadric * x);

        this->edge = e;
        this->optimalPoint = Vector3D(x.x, x.y, x.z);
        this->score = cost;
    }

  EdgeRecord HalfedgeMesh::quadricUpdate(EdgeIter e) {
      VertexIter onePoint = e->halfedge()->vertex();
      VertexIter otherPoint = e->halfedge()->twin()->vertex();

      e->quadric = onePoint->quadric + otherPoint->quadric;

      double data[9] = {};

      for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
              data[i * 3 + j] = e->quadric(i, j);
          }
      }

      Matrix3x3 A = Matrix3x3(data);
      Vector3D B = -1.0 * Vector3D(e->quadric(0, 3), e->quadric(1, 3), e->quadric(2, 3));

      // optimal point
      Vector3D x = A.inv() * B;

      double cost = dot(x, e->quadric * x);

      EdgeRecord record = EdgeRecord();
      record.edge = e;
      record.optimalPoint = x;
      record.score = cost;

      e->record = record;

      return record;
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
