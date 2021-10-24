#pragma once

#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "sol/sol.hpp"

struct Vector {
	float x, y, z;
	bool isInCuboid(const Vector& lower, const Vector& upper) const;
};

extern Vector* lineIntersectPos;

struct NodePoint {
	int x, y, z;

	bool operator==(const NodePoint& other) const {
		return x == other.x && y == other.y && z == other.z;
	}

	Vector toWorld() const { return {(float)x, (float)y - 0.001f, (float)z}; }

	Vector toWorldTop() const {
		return {(float)x, (float)y + 0.5f + 0.001f, (float)z};
	}

	bool isTouchingGround() const;

	bool isTouchingGroundWithLineOfSightFrom(Vector& otherPos,
	                                         Vector& otherPosTop) const;
};

struct Node {
	NodePoint point;
	std::unordered_set<unsigned int> linkIds;
	bool isOnStreet = false;
	Node(NodePoint& point) : point(point){};
};

typedef int (*lineIntersectLevelFunc)(Vector* posA, Vector* posB,
                                      int includeCityObjects);
extern lineIntersectLevelFunc lineIntersectLevel;

class NavGraphGenerator {
	std::vector<std::tuple<Vector, Vector>> streetCuboids;
	std::vector<Node> generatedNodes;
	int getHighestStreetY() const;

 public:
	void generate(int rootX, int rootY, int rootZ, sol::this_state s);
	void addStreetCuboid(Vector* lower, Vector* upper);
	void markStreetNodes();
	void write(std::string fileName) const;
};