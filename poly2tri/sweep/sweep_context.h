/*
 * Poly2Tri Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SWEEP_CONTEXT_H
#define SWEEP_CONTEXT_H

#include <list>
#include <vector>
#include <cstddef>

namespace p2t {

	// Inital triangle factor, seed triangle will extend 30% of
	// PointSet width to both left and right.
	const double kAlpha = 0.3;

	struct Point;
	class Triangle;
	struct Node;
	struct Edge;
	class AdvancingFront;

	class SweepContext {
	public:

		/// Constructor
		SweepContext(std::vector<Point*> polyline) :
			front_(0),
			head_(0),
			tail_(0),
			af_head_(0),
			af_middle_(0),
			af_tail_(0)
		{
			basin = Basin();
			edge_event = EdgeEvent();

			points_ = polyline;

			InitEdges(points_);
		}

		/// Destructor
		~SweepContext()
		{
			// Clean up memory

			delete head_;
			delete tail_;
			delete front_;
			delete af_head_;
			delete af_middle_;
			delete af_tail_;

			typedef std::list<Triangle*> type_list;

			for (type_list::iterator iter = map_.begin(); iter != map_.end(); ++iter) {
				Triangle* ptr = *iter;
				delete ptr;
			}

			for (unsigned int i = 0; i < edge_list.size(); i++) {
				delete edge_list[i];
			}
		}


		void set_head(Point* p1);

		Point* head();

		void set_tail(Point* p1);

		Point* tail();

		int point_count();

		Node& LocateNode(Point& point)
		{
			// TODO implement search tree
			return *front_->LocateNode(point.x);
		}


		void RemoveNode(Node* node)
		{
			delete node;
		}


		void CreateAdvancingFront(std::vector<Node*> nodes)
		{

			(void)nodes;
			// Initial triangle
			Triangle* triangle = new Triangle(*points_[0], *tail_, *head_);

			map_.push_back(triangle);

			af_head_ = new Node(*triangle->GetPoint(1), *triangle);
			af_middle_ = new Node(*triangle->GetPoint(0), *triangle);
			af_tail_ = new Node(*triangle->GetPoint(2));
			front_ = new AdvancingFront(*af_head_, *af_tail_);

			// TODO: More intuitive if head is middles next and not previous?
			//       so swap head and tail
			af_head_->next = af_middle_;
			af_middle_->next = af_tail_;
			af_middle_->prev = af_head_;
			af_tail_->prev = af_middle_;
		}


		/// Try to map a node to all sides of this triangle that don't have a neighbor
		void MapTriangleToNodes(Triangle& t)
		{
			for (int i = 0; i < 3; i++) {
				if (!t.GetNeighbor(i)) {
					Node* n = front_->LocatePoint(t.PointCW(*t.GetPoint(i)));
					if (n)
						n->triangle = &t;
				}
			}
		}


		void AddToMap(Triangle* triangle)
		{
			map_.push_back(triangle);
		}


		Point* GetPoint(const int& index)
		{
			return points_[index];
		}


		Point* GetPoints();

		void RemoveFromMap(Triangle* triangle)
		{
			map_.remove(triangle);
		}


		void AddHole(std::vector<Point*> polyline)
		{
			InitEdges(polyline);
			for (unsigned int i = 0; i < polyline.size(); i++) {
				points_.push_back(polyline[i]);
			}
		}


		void AddPoint(Point* point)
		{
			points_.push_back(point);
		}


		AdvancingFront* front();

		void MeshClean(Triangle& triangle)
		{
			std::vector<Triangle *> triangles;
			triangles.push_back(&triangle);

			while (!triangles.empty()) {
				Triangle *t = triangles.back();
				triangles.pop_back();

				if (t != NULL && !t->IsInterior()) {
					t->IsInterior(true);
					triangles_.push_back(t);
					for (int i = 0; i < 3; i++) {
						if (!t->constrained_edge[i])
							triangles.push_back(t->GetNeighbor(i));
					}
				}
			}
		}


		std::vector<Triangle*> GetTriangles()
		{
			return triangles_;
		}

		std::list<Triangle*> GetMap()
		{
			return map_;
		}


		std::vector<Edge*> edge_list;

		struct Basin {
			Node* left_node;
			Node* bottom_node;
			Node* right_node;
			double width;
			bool left_highest;

			Basin() : left_node(NULL), bottom_node(NULL), right_node(NULL), width(0.0), left_highest(false)
			{
			}

			void Clear()
			{
				left_node = NULL;
				bottom_node = NULL;
				right_node = NULL;
				width = 0.0;
				left_highest = false;
			}
		};

		struct EdgeEvent {
			Edge* constrained_edge;
			bool right;

			EdgeEvent() : constrained_edge(NULL), right(false)
			{
			}
		};

		Basin basin;
		EdgeEvent edge_event;

	private:

		friend class Sweep;

		std::vector<Triangle*> triangles_;
		std::list<Triangle*> map_;
		std::vector<Point*> points_;

		// Advancing front
		AdvancingFront* front_;
		// head point used with advancing front
		Point* head_;
		// tail point used with advancing front
		Point* tail_;

		Node *af_head_, *af_middle_, *af_tail_;

		void InitTriangulation()
		{
			double xmax(points_[0]->x), xmin(points_[0]->x);
			double ymax(points_[0]->y), ymin(points_[0]->y);

			// Calculate bounds.
			for (unsigned int i = 0; i < points_.size(); i++) {
				Point& p = *points_[i];
				if (p.x > xmax)
					xmax = p.x;
				if (p.x < xmin)
					xmin = p.x;
				if (p.y > ymax)
					ymax = p.y;
				if (p.y < ymin)
					ymin = p.y;
			}

			double dx = kAlpha * (xmax - xmin);
			double dy = kAlpha * (ymax - ymin);
			head_ = new Point(xmax + dx, ymin - dy);
			tail_ = new Point(xmin - dx, ymin - dy);

			// Sort points along y-axis
			std::sort(points_.begin(), points_.end(), cmp);

		}

		void InitEdges(std::vector<Point*> polyline)
		{
			int num_points = polyline.size();
			for (int i = 0; i < num_points; i++) {
				int j = i < num_points - 1 ? i + 1 : 0;
				edge_list.push_back(new Edge(*polyline[i], *polyline[j]));
			}
		}


	};

	inline AdvancingFront* SweepContext::front()
	{
		return front_;
	}

	inline int SweepContext::point_count()
	{
		return points_.size();
	}

	inline void SweepContext::set_head(Point* p1)
	{
		head_ = p1;
	}

	inline Point* SweepContext::head()
	{
		return head_;
	}

	inline void SweepContext::set_tail(Point* p1)
	{
		tail_ = p1;
	}

	inline Point* SweepContext::tail()
	{
		return tail_;
	}

}

#endif
